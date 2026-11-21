#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "tracker.h"

#include "assignment.h"

#ifndef MEASUREMENT_NOISE
#define MEASUREMENT_NOISE 4.0
#endif

/* Legacy diagonal process noise (Q_MODEL 0) */
#ifndef PROCESS_NOISE
#define PROCESS_NOISE 0.05
#endif

/*
 * Q_MODEL 1: constant-velocity model with random acceleration
 * (continuous white-noise acceleration). ACCEL_NOISE is the spectral
 * density in m^2/s^3.
 */
#ifndef Q_MODEL
#define Q_MODEL 1
#endif

#ifndef ACCEL_NOISE
#define ACCEL_NOISE 1.0
#endif

/* Association settings (real-data cluster tracking) */
#ifndef USE_HUNGARIAN
#define USE_HUNGARIAN 1          /* 1 = optimal assignment, 0 = greedy */
#endif

#ifndef USE_MAHALANOBIS
#define USE_MAHALANOBIS 1        /* 1 = covariance gate, 0 = metric gate */
#endif

#ifndef GATE_CHI2
#define GATE_CHI2 9.21           /* 99% gate for 2 degrees of freedom */
#endif

#ifndef CONFIRM_HITS
#define CONFIRM_HITS 3           /* detections needed to confirm a track */
#endif

#ifndef TENTATIVE_MAX_GAP
#define TENTATIVE_MAX_GAP 0.5    /* seconds before an unconfirmed track dies */
#endif

#ifndef SENSOR_FUSION
#define SENSOR_FUSION 1          /* 1 = tracks may be updated by any sensor */
#endif

#ifndef DOPPLER_INIT
#define DOPPLER_INIT 0           /* 1 = start velocity from Doppler speed */
#endif

#define NO_PAIR_COST 1.0e6

static int tracks_created_total = 0;
static int tracks_confirmed_total = 0;

void tracker_reset_counters(void)
{
    tracks_created_total = 0;
    tracks_confirmed_total = 0;
}

int tracker_tracks_created(void)
{
    return tracks_created_total;
}

int tracker_tracks_confirmed(void)
{
    return tracks_confirmed_total;
}

#define MIN_DT 0.000001
#ifndef MAX_TRACK_GAP
#define MAX_TRACK_GAP 1.5
#endif

/* ==========================================
   TARGET SLOT MANAGEMENT
   ========================================== */

static int allocate_target_slot(Target targets[], int *count)
{
    for (int i = 0; i < *count; i++) {
        if (!targets[i].active) {
            return i;
        }
    }

    if (*count < MAX_TARGETS) {
        int slot = *count;
        (*count)++;
        return slot;
    }

    return -1;
}

/* ==========================================
   TARGET HISTORY
   ========================================== */

static void append_history(
    Target *target,
    double x,
    double y,
    double timestamp)
{
    if (target->history_count < MAX_HISTORY) {
        int index = target->history_count++;

        target->history[index].x = x;
        target->history[index].y = y;
        target->history[index].timestamp = timestamp;
    }
    else {
        for (int i = 1; i < MAX_HISTORY; i++) {
            target->history[i - 1] = target->history[i];
        }

        target->history[MAX_HISTORY - 1].x = x;
        target->history[MAX_HISTORY - 1].y = y;
        target->history[MAX_HISTORY - 1].timestamp = timestamp;
    }
}

/* ==========================================
   FIND TARGET
   ========================================== */

int find_target(
    const Target targets[],
    int count,
    int id)
{
    if (targets == NULL || count <= 0) {
        return -1;
    }

    for (int i = 0; i < count; i++) {
        if (targets[i].id == id) {
            return i;
        }
    }

    return -1;
}

/* ==========================================
   INITIALIZE KALMAN FILTER
   State: [x, y, vx, vy]
   ========================================== */

void initialize_kalman(Target *target)
{
    if (target == NULL) {
        return;
    }

    memset(&target->kalman, 0, sizeof(KalmanState));

    target->kalman.state[0] = target->x;
    target->kalman.state[1] = target->y;
    target->kalman.state[2] = 0.0;
    target->kalman.state[3] = 0.0;

    for (int i = 0; i < 4; i++) {
        target->kalman.covariance[i][i] = 10.0;
    }

    target->kalman.initialized = 1;

    target->vx = 0.0;
    target->vy = 0.0;
    target->speed = 0.0;
}

/* ==========================================
   KALMAN PREDICTION AND GATING HELPERS
   ========================================== */

static void kalman_predict(
    const Target *target,
    double dt,
    double predicted_state[4],
    double predicted_P[4][4])
{
    double F[4][4] = {
        {1.0, 0.0, dt,  0.0},
        {0.0, 1.0, 0.0, dt},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    double FP[4][4] = {{0}};

    for (int i = 0; i < 4; i++) {
        predicted_state[i] = 0.0;

        for (int j = 0; j < 4; j++) {
            predicted_P[i][j] = 0.0;
            predicted_state[i] += F[i][j] * target->kalman.state[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * target->kalman.covariance[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                predicted_P[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

#if Q_MODEL == 1
    {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double q = ACCEL_NOISE;

        predicted_P[0][0] += q * dt3 / 3.0;
        predicted_P[1][1] += q * dt3 / 3.0;
        predicted_P[0][2] += q * dt2 / 2.0;
        predicted_P[2][0] += q * dt2 / 2.0;
        predicted_P[1][3] += q * dt2 / 2.0;
        predicted_P[3][1] += q * dt2 / 2.0;
        predicted_P[2][2] += q * dt;
        predicted_P[3][3] += q * dt;
    }
#else
    for (int i = 0; i < 4; i++) {
        predicted_P[i][i] += PROCESS_NOISE * dt;
    }
#endif
}

/* Squared Mahalanobis distance between a measurement and a track. */
static double kalman_mahalanobis_sq(
    const Target *target,
    double measured_x,
    double measured_y,
    double dt)
{
    double state[4];
    double P[4][4];

    kalman_predict(target, dt, state, P);

    double nx = measured_x - state[0];
    double ny = measured_y - state[1];

    double S00 = P[0][0] + MEASUREMENT_NOISE;
    double S01 = P[0][1];
    double S10 = P[1][0];
    double S11 = P[1][1] + MEASUREMENT_NOISE;

    double det = S00 * S11 - S01 * S10;

    if (fabs(det) < 1e-12) {
        return NO_PAIR_COST;
    }

    return (nx * (S11 * nx - S01 * ny) +
            ny * (-S10 * nx + S00 * ny)) / det;
}

/* ==========================================
   KALMAN FILTER
   State: [x, y, vx, vy]
   Measurement: [x, y]
   ========================================== */

void update_kalman(
    Target *target,
    double measured_x,
    double measured_y,
    double dt)
{
    if (target == NULL) {
        return;
    }

    if (!target->kalman.initialized) {
        target->x = measured_x;
        target->y = measured_y;

        initialize_kalman(target);
        return;
    }

    if (dt <= MIN_DT) {
        return;
    }

    double predicted_P[4][4];
    double predicted_state[4];

    kalman_predict(target, dt, predicted_state, predicted_P);

    double innovation_x =
        measured_x - predicted_state[0];

    double innovation_y =
        measured_y - predicted_state[1];

    double S00 = predicted_P[0][0] + MEASUREMENT_NOISE;
    double S01 = predicted_P[0][1];
    double S10 = predicted_P[1][0];
    double S11 = predicted_P[1][1] + MEASUREMENT_NOISE;

    double determinant = S00 * S11 - S01 * S10;

    if (fabs(determinant) < 1e-12) {
        return;
    }

    double invS[2][2];

    invS[0][0] = S11 / determinant;
    invS[0][1] = -S01 / determinant;
    invS[1][0] = -S10 / determinant;
    invS[1][1] = S00 / determinant;

    double K[4][2] = {{0}};

    for (int i = 0; i < 4; i++) {
        K[i][0] =
            predicted_P[i][0] * invS[0][0] +
            predicted_P[i][1] * invS[1][0];

        K[i][1] =
            predicted_P[i][0] * invS[0][1] +
            predicted_P[i][1] * invS[1][1];
    }

    for (int i = 0; i < 4; i++) {
        target->kalman.state[i] =
            predicted_state[i] +
            K[i][0] * innovation_x +
            K[i][1] * innovation_y;
    }

    double updated_P[4][4] = {{0}};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            updated_P[i][j] =
                predicted_P[i][j] -
                K[i][0] * predicted_P[0][j] -
                K[i][1] * predicted_P[1][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            target->kalman.covariance[i][j] =
                updated_P[i][j];
        }
    }

    target->vx = target->kalman.state[2];
    target->vy = target->kalman.state[3];

    target->speed = sqrt(
        target->vx * target->vx +
        target->vy * target->vy
    );
}

/* ==========================================
   ADD OR UPDATE TARGET
   ========================================== */

int update_target(
    Target targets[],
    int *count,
    int id,
    double x,
    double y,
    double timestamp)
{
    if (targets == NULL || count == NULL ||
        *count < 0 || *count > MAX_TARGETS) {
        return 0;
    }

    int index = find_target(targets, *count, id);

    if (index == -1) {
        index = allocate_target_slot(targets, count);

        if (index < 0) {
            printf("No free target slot is available.\n");
            return 0;
        }

        memset(&targets[index], 0, sizeof(Target));

        targets[index].id = id;
        targets[index].x = x;
        targets[index].y = y;
        targets[index].timestamp = timestamp;
        targets[index].active = 1;
        targets[index].confirmed = 1;
        targets[index].detection_count = 1;

        initialize_kalman(&targets[index]);
        append_history(&targets[index], x, y, timestamp);

        return 1;
    }

    Target *target = &targets[index];

    if (!target->active) {
        memset(target, 0, sizeof(Target));

        target->id = id;
        target->x = x;
        target->y = y;
        target->timestamp = timestamp;
        target->active = 1;
        target->confirmed = 1;
        target->detection_count = 1;

        initialize_kalman(target);
        append_history(target, x, y, timestamp);

        return 1;
    }

    double dt = timestamp - target->timestamp;

    if (dt <= MIN_DT) {
        return 0;
    }

    double predicted_x =
        target->kalman.state[0] +
        target->kalman.state[2] * dt;

    double predicted_y =
        target->kalman.state[1] +
        target->kalman.state[3] * dt;

    double error_x = x - predicted_x;
    double error_y = y - predicted_y;

    target->prediction_error =
        sqrt(error_x * error_x + error_y * error_y);

    target->total_prediction_error +=
        target->prediction_error;

    target->prediction_count++;
    target->has_prediction_error = 1;

    update_kalman(target, x, y, dt);

    target->x = x;
    target->y = y;
    target->timestamp = timestamp;
    target->active = 1;

    target->missed_frames = 0;
    target->detection_count++;

    append_history(target, x, y, timestamp);

    return 1;
}

/* ==========================================
   DISPLAY TARGET INFORMATION
   ========================================== */

void display_targets(
    const Target targets[],
    int count)
{
    printf("\n========== TARGET DETAILS ==========\n");

    for (int i = 0; i < count; i++) {
        printf("\nTarget ID: %d\n", targets[i].id);

        printf("Measured Position: (%.2f, %.2f)\n",
               targets[i].x, targets[i].y);

        printf("Kalman Position: (%.2f, %.2f)\n",
               targets[i].kalman.state[0],
               targets[i].kalman.state[1]);

        printf("Filtered Velocity: (%.2f, %.2f)\n",
               targets[i].vx, targets[i].vy);

        printf("Speed: %.2f\n", targets[i].speed);

        printf("Detection Count: %d\n",
               targets[i].detection_count);

        printf("Status: %s\n",
               targets[i].active ? "Active" : "Inactive");
    }
}

/* ==========================================
   DISPLAY TARGET HISTORY
   ========================================== */

void display_target_history(
    const Target targets[],
    int count,
    int id)
{
    int index = find_target(targets, count, id);

    if (index == -1) {
        printf("Target not found.\n");
        return;
    }

    printf("\nHistory for Target %d:\n", id);

    for (int i = 0; i < targets[index].history_count; i++) {
        printf(
            "Time: %.4f | Position: (%.2f, %.2f)\n",
            targets[index].history[i].timestamp,
            targets[index].history[i].x,
            targets[index].history[i].y
        );
    }
}

/* ==========================================
   FUTURE POSITION PREDICTION
   ========================================== */

void display_prediction(
    const Target targets[],
    int count,
    double future_seconds)
{
    printf("\n========== FUTURE POSITION PREDICTION ==========\n");

    for (int i = 0; i < count; i++) {
        if (!targets[i].active) {
            continue;
        }

        double future_x =
            targets[i].kalman.state[0] +
            targets[i].kalman.state[2] * future_seconds;

        double future_y =
            targets[i].kalman.state[1] +
            targets[i].kalman.state[3] * future_seconds;

        printf(
            "Target %d: Future Position = (%.2f, %.2f)\n",
            targets[i].id,
            future_x,
            future_y
        );
    }
}

/* ==========================================
   POSITION REFLECTION FOR SIMULATION
   ========================================== */

static double reflect_position(double position)
{
    while (position < 0.0 || position > 100.0) {
        if (position < 0.0) {
            position = -position;
        }

        if (position > 100.0) {
            position = 200.0 - position;
        }
    }

    return position;
}

/* ==========================================
   SIMULATE TARGET MOVEMENT
   ========================================== */

void simulate_target_step(
    Target targets[],
    int *count,
    double time_step)
{
    int original_count = *count;

    for (int i = 0; i < original_count; i++) {
        if (!targets[i].active) {
            continue;
        }

        int id = targets[i].id;

        double next_x =
            targets[i].x + targets[i].vx * time_step;

        double next_y =
            targets[i].y + targets[i].vy * time_step;

        next_x = reflect_position(next_x);
        next_y = reflect_position(next_y);

        double next_timestamp =
            targets[i].timestamp + time_step;

        update_target(
            targets,
            count,
            id,
            next_x,
            next_y,
            next_timestamp
        );
    }
}

/* ==========================================
   TRACKING ACCURACY REPORT
   ========================================== */

void display_tracking_accuracy(
    const Target targets[],
    int count)
{
    printf("\n========== TRACKING ERROR REPORT ==========\n");

    for (int i = 0; i < count; i++) {
        double average_error = 0.0;

        if (targets[i].prediction_count > 0) {
            average_error =
                targets[i].total_prediction_error /
                targets[i].prediction_count;
        }

        printf("\nTarget ID: %d\n", targets[i].id);

        if (targets[i].has_prediction_error) {
            printf(
                "Latest Prediction Error: %.2f\n",
                targets[i].prediction_error
            );
        } else {
            printf("Latest Prediction Error: Not available\n");
        }

        printf("Average Prediction Error: %.2f\n", average_error);

        printf("Prediction Count: %d\n",
               targets[i].prediction_count);
    }
}

/* ==========================================
   JSON EXPORT HELPER
   ========================================== */

static void write_targets_json(
    FILE *file,
    const Target targets[],
    int count)
{
    int active_total = 0;

    for (int i = 0; i < count; i++) {
        if (targets[i].active && targets[i].confirmed) {
            active_total++;
        }
    }

    fprintf(file, "{\n");
    fprintf(file, "    \"total_targets\": %d,\n", active_total);
    fprintf(file, "    \"targets\": [\n");

    int written = 0;

    for (int i = 0; i < count; i++) {
        if (!targets[i].active || !targets[i].confirmed) {
            continue;
        }

        double average_error = 0.0;

        if (targets[i].prediction_count > 0) {
            average_error =
                targets[i].total_prediction_error /
                targets[i].prediction_count;
        }

        fprintf(file, "        {\n");

        fprintf(file, "            \"id\": %d,\n", targets[i].id);
        fprintf(file, "            \"x\": %.4f,\n", targets[i].x);
        fprintf(file, "            \"y\": %.4f,\n", targets[i].y);
        fprintf(file, "            \"vx\": %.4f,\n", targets[i].vx);
        fprintf(file, "            \"vy\": %.4f,\n", targets[i].vy);
        fprintf(file, "            \"speed\": %.4f,\n", targets[i].speed);
        fprintf(file, "            \"range\": %.4f,\n", targets[i].range);
        fprintf(file, "            \"azimuth\": %.6f,\n", targets[i].azimuth);
        fprintf(file, "            \"rcs\": %.4f,\n", targets[i].rcs);

        fprintf(file, "            \"radial_velocity\": %.4f,\n",
                targets[i].radial_velocity);

        fprintf(file, "            \"kalman_x\": %.4f,\n",
                targets[i].kalman.state[0]);

        fprintf(file, "            \"kalman_y\": %.4f,\n",
                targets[i].kalman.state[1]);

        fprintf(file, "            \"kalman_vx\": %.4f,\n",
                targets[i].kalman.state[2]);

        fprintf(file, "            \"kalman_vy\": %.4f,\n",
                targets[i].kalman.state[3]);

        fprintf(file, "            \"prediction_error\": %.4f,\n",
                targets[i].prediction_error);

        fprintf(file, "            \"average_prediction_error\": %.4f,\n",
                average_error);

        fprintf(file, "            \"detection_count\": %d,\n",
                targets[i].detection_count);

        fprintf(file, "            \"sensor_id\": %d,\n",
                targets[i].sensor_id);

        fprintf(file, "            \"label_id\": %d,\n",
                targets[i].label_id);

        fprintf(file, "            \"reference_track_id\": \"%s\",\n",
                targets[i].reference_track_id);

        fprintf(file, "            \"active\": true\n");

        written++;

        if (written < active_total) {
            fprintf(file, "        },\n");
        } else {
            fprintf(file, "        }\n");
        }
    }

    fprintf(file, "    ]\n");
    fprintf(file, "}\n");
}

/* ==========================================
   EXPORT JSON
   ========================================== */

void export_targets_json(
    const Target targets[],
    int count)
{
    FILE *root_file = fopen("targets.json", "w");

    FILE *dashboard_file =
        fopen("dashboard/targets.json", "w");

    if (root_file != NULL) {
        write_targets_json(root_file, targets, count);
        fclose(root_file);
    } else {
        printf("Could not create targets.json\n");
    }

    if (dashboard_file != NULL) {
        write_targets_json(dashboard_file, targets, count);
        fclose(dashboard_file);
    } else {
        printf("Could not create dashboard/targets.json\n");
    }
}

/* ==========================================
   TRACK LIFECYCLE MANAGEMENT
   ========================================== */

static void expire_stale_tracks(
    Target targets[],
    int count,
    double timestamp)
{
    for (int i = 0; i < count; i++) {
        if (!targets[i].active) {
            continue;
        }

        double age = timestamp - targets[i].timestamp;

        double allowed_gap =
            targets[i].confirmed ? MAX_TRACK_GAP : TENTATIVE_MAX_GAP;

        if (age > allowed_gap) {
            targets[i].active = 0;
            targets[i].missed_frames++;
        }
    }
}

/* ==========================================
   REAL RADAR DETECTION ASSOCIATION
   Uses Kalman-predicted position
   ========================================== */

int update_radar_detection(
    Target targets[],
    int *count,
    int *next_track_id,
    int sensor_id,
    double range,
    double azimuth,
    double rcs,
    double radial_velocity,
    double x,
    double y,
    double timestamp,
    const char *uuid,
    const char *reference_track_id,
    int label_id,
    double gate_distance)
{
    if (targets == NULL ||
        count == NULL ||
        next_track_id == NULL ||
        *count < 0 ||
        *count > MAX_TARGETS ||
        gate_distance <= 0.0) {
        return 0;
    }

    expire_stale_tracks(targets, *count, timestamp);

    int best_index = -1;
    double best_distance = gate_distance;

    for (int i = 0; i < *count; i++) {
        Target *target = &targets[i];

        if (!target->active) {
            continue;
        }

        if (target->sensor_id != sensor_id) {
            continue;
        }

        double dt = timestamp - target->timestamp;

        if (dt <= MIN_DT || dt > MAX_TRACK_GAP) {
            continue;
        }

        double predicted_x =
            target->kalman.state[0] +
            target->kalman.state[2] * dt;

        double predicted_y =
            target->kalman.state[1] +
            target->kalman.state[3] * dt;

        double dx = x - predicted_x;
        double dy = y - predicted_y;

        double distance = sqrt(dx * dx + dy * dy);

        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }

    int track_id;

    if (best_index >= 0) {
        track_id = targets[best_index].id;
    } else {
        while (find_target(
            targets,
            *count,
            *next_track_id
        ) >= 0) {
            (*next_track_id)++;
        }

        track_id = *next_track_id;
        (*next_track_id)++;
    }

    int result = update_target(
        targets,
        count,
        track_id,
        x,
        y,
        timestamp
    );

    if (!result) {
        return 0;
    }

    int index = find_target(targets, *count, track_id);

    if (index < 0) {
        return 0;
    }

    Target *target = &targets[index];

    target->sensor_id = sensor_id;
    target->range = range;
    target->azimuth = azimuth;
    target->rcs = rcs;
    target->radial_velocity = radial_velocity;
    target->label_id = label_id;

    target->missed_frames = 0;

    if (uuid != NULL) {
        snprintf(
            target->uuid,
            sizeof(target->uuid),
            "%s",
            uuid
        );
    }

    if (reference_track_id != NULL &&
        reference_track_id[0] != '\0') {
        snprintf(
            target->reference_track_id,
            sizeof(target->reference_track_id),
            "%s",
            reference_track_id
        );
    }

    return track_id;
}

/* ==========================================
   FRAME-LEVEL ONE-TO-ONE DATA ASSOCIATION
   ========================================== */

typedef struct {
    int track_index;
    int detection_index;
    double distance;
} AssociationCandidate;

static int compare_association_candidates(
    const void *left,
    const void *right)
{
    const AssociationCandidate *a =
        (const AssociationCandidate *)left;

    const AssociationCandidate *b =
        (const AssociationCandidate *)right;

    if (a->distance < b->distance) {
        return -1;
    }

    if (a->distance > b->distance) {
        return 1;
    }

    return 0;
}

/* ==========================================
   UPDATE RADAR FRAME
   Global greedy nearest-neighbour matching
   ========================================== */

int update_radar_frame(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarDetection detections[],
    int detection_count,
    double gate_distance)
{
    if (targets == NULL ||
        count == NULL ||
        next_track_id == NULL ||
        detections == NULL ||
        detection_count <= 0 ||
        gate_distance <= 0.0 ||
        *count < 0 ||
        *count > MAX_TARGETS) {
        return 0;
    }

    int track_capacity = *count;

    size_t max_pairs =
        (size_t)track_capacity *
        (size_t)detection_count;

    AssociationCandidate *candidates = NULL;

    int *track_used = calloc(
        (size_t)(track_capacity > 0 ? track_capacity : 1),
        sizeof(int)
    );

    int *detection_used = calloc(
        (size_t)detection_count,
        sizeof(int)
    );

    if (max_pairs > 0) {
        candidates = malloc(
            max_pairs * sizeof(AssociationCandidate)
        );
    }

    if ((max_pairs > 0 && candidates == NULL) ||
        track_used == NULL ||
        detection_used == NULL) {

        free(candidates);
        free(track_used);
        free(detection_used);

        return 0;
    }

    size_t candidate_count = 0;

    double timestamp = detections[0].timestamp;

    expire_stale_tracks(targets, *count, timestamp);

    for (int ti = 0; ti < track_capacity; ti++) {
        Target *target = &targets[ti];

        if (!target->active) {
            continue;
        }

        for (int di = 0; di < detection_count; di++) {
            const RadarDetection *d = &detections[di];

            if (d->sensor_id != target->sensor_id) {
                continue;
            }

            if (fabs(d->timestamp - timestamp) > 1e-9) {
                continue;
            }

            double dt = d->timestamp - target->timestamp;

            if (dt <= MIN_DT || dt > MAX_TRACK_GAP) {
                continue;
            }

            double predicted_x =
                target->kalman.state[0] +
                target->kalman.state[2] * dt;

            double predicted_y =
                target->kalman.state[1] +
                target->kalman.state[3] * dt;

            double dx = d->x - predicted_x;
            double dy = d->y - predicted_y;

            double distance = sqrt(dx * dx + dy * dy);

            if (distance <= gate_distance) {
                candidates[candidate_count].track_index = ti;
                candidates[candidate_count].detection_index = di;
                candidates[candidate_count].distance = distance;

                candidate_count++;
            }
        }
    }

    if (candidate_count > 1) {
        qsort(
            candidates,
            candidate_count,
            sizeof(AssociationCandidate),
            compare_association_candidates
        );
    }

    int processed = 0;

    for (size_t i = 0; i < candidate_count; i++) {
        int ti = candidates[i].track_index;
        int di = candidates[i].detection_index;

        if (track_used[ti] || detection_used[di]) {
            continue;
        }

        Target *target = &targets[ti];
        const RadarDetection *d = &detections[di];

        int id = target->id;

        int success = update_target(
            targets,
            count,
            id,
            d->x,
            d->y,
            d->timestamp
        );

        if (!success) {
            continue;
        }

        int updated_index = find_target(targets, *count, id);

        if (updated_index < 0) {
            continue;
        }

        target = &targets[updated_index];

        target->sensor_id = d->sensor_id;
        target->range = d->range;
        target->azimuth = d->azimuth;
        target->rcs = d->rcs;
        target->radial_velocity = d->compensated_velocity;
        target->label_id = d->label_id;
        target->missed_frames = 0;

        snprintf(
            target->uuid,
            sizeof(target->uuid),
            "%s",
            d->uuid
        );

        if (d->reference_track_id[0] != '\0') {
            snprintf(
                target->reference_track_id,
                sizeof(target->reference_track_id),
                "%s",
                d->reference_track_id
            );
        }

        track_used[ti] = 1;
        detection_used[di] = 1;

        processed++;
    }

    for (int di = 0; di < detection_count; di++) {
        if (detection_used[di]) {
            continue;
        }

        const RadarDetection *d = &detections[di];

        int result = update_radar_detection(
            targets,
            count,
            next_track_id,
            d->sensor_id,
            d->range,
            d->azimuth,
            d->rcs,
            d->compensated_velocity,
            d->x,
            d->y,
            d->timestamp,
            d->uuid,
            d->reference_track_id,
            d->label_id,
            gate_distance
        );

        if (result > 0) {
            processed++;
        }
    }

    free(candidates);
    free(track_used);
    free(detection_used);

    return processed;
}

/* ==========================================
   CLUSTER-LEVEL RADAR TRACKING
   Global greedy nearest-neighbour association
   ========================================== */

int update_radar_clusters(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarCluster clusters[],
    int cluster_count,
    int cluster_track_ids[],
    double gate_distance
)
{
    if (targets == NULL ||
        count == NULL ||
        next_track_id == NULL ||
        clusters == NULL ||
        cluster_track_ids == NULL ||
        cluster_count <= 0 ||
        gate_distance <= 0.0 ||
        *count < 0 ||
        *count > MAX_TARGETS) {
        return 0;
    }

    for (int i = 0; i < cluster_count; i++) {
        cluster_track_ids[i] = -1;
    }

    double frame_timestamp = clusters[0].timestamp;

    expire_stale_tracks(targets, *count, frame_timestamp);

    /* ---- STEP 1: list the tracks that can take part in this frame ---- */

    int track_capacity = *count;

    int *eligible = malloc(
        (size_t)(track_capacity > 0 ? track_capacity : 1) * sizeof(int)
    );

    int *cluster_used = calloc((size_t)cluster_count, sizeof(int));

    if (eligible == NULL || cluster_used == NULL) {
        free(eligible);
        free(cluster_used);
        return 0;
    }

    int eligible_count = 0;

    for (int ti = 0; ti < track_capacity; ti++) {
        const Target *target = &targets[ti];

        if (!target->active) {
            continue;
        }

        double dt = frame_timestamp - target->timestamp;

        if (dt <= MIN_DT || dt > MAX_TRACK_GAP) {
            continue;
        }

        eligible[eligible_count++] = ti;
    }

    int processed = 0;

    /* ---- STEP 2: gated cost matrix and one-to-one assignment ---- */

    if (eligible_count > 0) {

        double *cost = malloc(
            (size_t)eligible_count * (size_t)cluster_count * sizeof(double)
        );

        int *row_to_col = malloc((size_t)eligible_count * sizeof(int));

        if (cost == NULL || row_to_col == NULL) {
            free(cost);
            free(row_to_col);
            free(eligible);
            free(cluster_used);
            return 0;
        }

        for (int r = 0; r < eligible_count; r++) {
            const Target *target = &targets[eligible[r]];
            double dt = frame_timestamp - target->timestamp;

            for (int ci = 0; ci < cluster_count; ci++) {
                const RadarCluster *cluster = &clusters[ci];
                double c = NO_PAIR_COST;

#if !SENSOR_FUSION
                int same_sensor = (cluster->sensor_id == target->sensor_id);
#else
                int same_sensor = 1;
#endif

                if (same_sensor &&
                    fabs(cluster->timestamp - frame_timestamp) <= 1e-9) {

#if USE_MAHALANOBIS
                    double d2 = kalman_mahalanobis_sq(
                        target, cluster->x, cluster->y, dt
                    );

                    if (d2 <= GATE_CHI2) {
                        c = d2;
                    }
#else
                    double px = target->kalman.state[0] +
                                target->kalman.state[2] * dt;
                    double py = target->kalman.state[1] +
                                target->kalman.state[3] * dt;
                    double dx = cluster->x - px;
                    double dy = cluster->y - py;
                    double distance = sqrt(dx * dx + dy * dy);

                    if (distance <= gate_distance) {
                        c = distance;
                    }
#endif
                }

                cost[(size_t)r * (size_t)cluster_count + (size_t)ci] = c;
            }
        }

#if USE_HUNGARIAN
        hungarian_assign(cost, eligible_count, cluster_count, row_to_col);
#else
        /* Greedy nearest-first assignment (previous behaviour). */
        for (int r = 0; r < eligible_count; r++) {
            row_to_col[r] = -1;
        }

        {
            int *col_taken = calloc((size_t)cluster_count, sizeof(int));
            int *row_taken = calloc((size_t)eligible_count, sizeof(int));

            if (col_taken != NULL && row_taken != NULL) {
                for (;;) {
                    double best = NO_PAIR_COST;
                    int best_r = -1;
                    int best_c = -1;

                    for (int r = 0; r < eligible_count; r++) {
                        if (row_taken[r]) {
                            continue;
                        }

                        for (int ci = 0; ci < cluster_count; ci++) {
                            double c = cost[(size_t)r * (size_t)cluster_count + (size_t)ci];

                            if (!col_taken[ci] && c < best) {
                                best = c;
                                best_r = r;
                                best_c = ci;
                            }
                        }
                    }

                    if (best_r < 0) {
                        break;
                    }

                    row_to_col[best_r] = best_c;
                    row_taken[best_r] = 1;
                    col_taken[best_c] = 1;
                }
            }

            free(col_taken);
            free(row_taken);
        }
#endif

        /* ---- STEP 3: apply the accepted assignments ---- */

        for (int r = 0; r < eligible_count; r++) {
            int ci = row_to_col[r];

            if (ci < 0 ||
                cost[(size_t)r * (size_t)cluster_count + (size_t)ci] >= NO_PAIR_COST) {
                continue;
            }

            int ti = eligible[r];
            const RadarCluster *cluster = &clusters[ci];
            int track_id = targets[ti].id;

            if (!update_target(
                    targets, count, track_id,
                    cluster->x, cluster->y, cluster->timestamp)) {
                continue;
            }

            int index = find_target(targets, *count, track_id);

            if (index < 0) {
                continue;
            }

            Target *target = &targets[index];

            target->sensor_id = cluster->sensor_id;
            target->range = cluster->range;
            target->azimuth = cluster->azimuth;
            target->rcs = cluster->rcs;
            target->radial_velocity = cluster->radial_velocity;
            target->missed_frames = 0;

            if (cluster->reference_track_id[0] != '\0') {
                snprintf(
                    target->reference_track_id,
                    sizeof(target->reference_track_id),
                    "%s",
                    cluster->reference_track_id
                );
            }

            if (!target->confirmed &&
                target->detection_count >= CONFIRM_HITS) {
                target->confirmed = 1;
                tracks_confirmed_total++;
            }

            cluster_track_ids[ci] = track_id;
            cluster_used[ci] = 1;
            processed++;
        }

        free(cost);
        free(row_to_col);
    }

    free(eligible);

    /* ---- STEP 4: unmatched clusters start new (tentative) tracks ---- */

    for (int ci = 0; ci < cluster_count; ci++) {

        if (cluster_used[ci]) {
            continue;
        }

        const RadarCluster *cluster = &clusters[ci];

        /*
         * A cluster very close to a track that was already updated in this
         * frame is most likely a split of that object: do not start a track.
         */
        int duplicate = 0;

        for (int ti = 0; ti < *count; ti++) {
            const Target *target = &targets[ti];

            if (!target->active ||
                fabs(target->timestamp - frame_timestamp) > 1e-9) {
                continue;
            }

#if !SENSOR_FUSION
            if (target->sensor_id != cluster->sensor_id) {
                continue;
            }
#endif

            double dx = cluster->x - target->x;
            double dy = cluster->y - target->y;

            if (sqrt(dx * dx + dy * dy) <= gate_distance) {
                duplicate = 1;
                break;
            }
        }

        if (duplicate) {
            continue;
        }

        while (find_target(targets, *count, *next_track_id) >= 0) {
            (*next_track_id)++;
        }

        int track_id = *next_track_id;

        if (!update_target(
                targets, count, track_id,
                cluster->x, cluster->y, cluster->timestamp)) {
            continue;
        }

        (*next_track_id)++;

        int index = find_target(targets, *count, track_id);

        if (index < 0) {
            continue;
        }

        Target *target = &targets[index];

        target->sensor_id = cluster->sensor_id;
        target->range = cluster->range;
        target->azimuth = cluster->azimuth;
        target->rcs = cluster->rcs;
        target->radial_velocity = cluster->radial_velocity;
        target->label_id = -1;
        target->missed_frames = 0;

        if (cluster->reference_track_id[0] != '\0') {
            snprintf(
                target->reference_track_id,
                sizeof(target->reference_track_id),
                "%s",
                cluster->reference_track_id
            );
        }

#if DOPPLER_INIT
        {
            double r = sqrt(cluster->x * cluster->x +
                            cluster->y * cluster->y);

            if (r > 1e-6) {
                double vr = cluster->radial_velocity;

                target->kalman.state[2] = vr * cluster->x / r;
                target->kalman.state[3] = vr * cluster->y / r;
                target->vx = target->kalman.state[2];
                target->vy = target->kalman.state[3];
                target->speed = fabs(vr);
            }
        }
#endif

        tracks_created_total++;

        if (CONFIRM_HITS <= 1) {
            target->confirmed = 1;
            tracks_confirmed_total++;
        }
        else {
            target->confirmed = 0;
        }

        cluster_track_ids[ci] = track_id;
        cluster_used[ci] = 1;
        processed++;
    }

    free(cluster_used);

    return processed;
}
