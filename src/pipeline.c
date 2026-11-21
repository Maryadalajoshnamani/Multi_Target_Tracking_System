#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pipeline.h"
#include "tracker.h"
#include "radar_reader.h"
#include "clustering.h"
#include "database.h"

#define DEFAULT_DATASET_PATH "data/radarscenes/sequence_151/radar_detections.csv"

#ifndef ASSOCIATION_GATE
#define ASSOCIATION_GATE 5.0
#endif

#define ASSOCIATION_LOG_PATH "radar_association_log_frame.csv"

static const char *dataset_path = DEFAULT_DATASET_PATH;
static int pipeline_verbose = 0;

void pipeline_set_dataset_path(const char *path)
{
    if (path != NULL && path[0] != '\0') {
        dataset_path = path;
    }
}

void pipeline_set_verbose(int verbose)
{
    pipeline_verbose = verbose;
}

/* ==========================================
   SAVE CLUSTER ASSOCIATION RESULTS
   ========================================== */

static void save_cluster_associations(
    FILE *log_file,
    const RadarDetection detections[],
    int detection_count,
    const int detection_cluster_ids[],
    const int cluster_track_ids[]
)
{
    if (log_file == NULL ||
        detections == NULL ||
        detection_cluster_ids == NULL ||
        cluster_track_ids == NULL) {
        return;
    }

    for (int i = 0; i < detection_count; i++) {

        int cluster_id = detection_cluster_ids[i];

        /* Ignore noise points and unclustered detections. */

        if (cluster_id < 0) {
            continue;
        }

        int internal_id = cluster_track_ids[cluster_id];

        if (internal_id <= 0) {
            continue;
        }

        const RadarDetection *detection = &detections[i];

        /* Only labelled detections are used for evaluation. */

        if (detection->reference_track_id[0] == '\0' ||
            detection->label_id == 11) {
            continue;
        }

        fprintf(
            log_file,
            "%.6f,%d,%s,%d,%d,%.6f,%.6f\n",
            detection->timestamp,
            detection->sensor_id,
            detection->reference_track_id,
            internal_id,
            detection->label_id,
            detection->x,
            detection->y
        );
    }
}

/* ==========================================
   PROCESS COMPLETE RADAR FRAME
   ========================================== */

static int process_frame_buffer(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarDetection frame[],
    int frame_count,
    FILE *log_file
)
{
    if (frame == NULL || frame_count <= 0) {
        return 0;
    }

    /* Allocate cluster memory. */

    RadarCluster *clusters = calloc(
        (size_t)frame_count,
        sizeof(RadarCluster)
    );

    int *detection_cluster_ids = malloc(
        (size_t)frame_count * sizeof(int)
    );

    int *cluster_track_ids = malloc(
        (size_t)frame_count * sizeof(int)
    );

    if (clusters == NULL ||
        detection_cluster_ids == NULL ||
        cluster_track_ids == NULL) {

        printf("Memory allocation failed during clustering.\n");

        free(clusters);
        free(detection_cluster_ids);
        free(cluster_track_ids);

        return 0;
    }

    /* Initialize mapping arrays. */

    for (int i = 0; i < frame_count; i++) {
        detection_cluster_ids[i] = -1;
        cluster_track_ids[i] = -1;
    }

    /*
       STEP 1:
       Cluster detections by spatial proximity.
       Clustering implementation separates sensors
       and timestamps.
    */

    int cluster_count = cluster_radar_detections(
        frame,
        frame_count,
        clusters,
        frame_count,
        detection_cluster_ids,
        CLUSTER_EPSILON,
        CLUSTER_MIN_POINTS
    );

    /*
       STEP 2:
       Associate cluster centroids with existing tracks.
    */

    int processed_clusters = 0;

    if (cluster_count > 0) {

        processed_clusters = update_radar_clusters(
            targets,
            count,
            next_track_id,
            clusters,
            cluster_count,
            cluster_track_ids,
            ASSOCIATION_GATE
        );
    }

    /*
       STEP 3:
       Count detections that belong to valid clusters.
    */

    int processed_detections = 0;

    for (int i = 0; i < frame_count; i++) {

        int cluster_id = detection_cluster_ids[i];

        if (cluster_id >= 0 &&
            cluster_id < cluster_count &&
            cluster_track_ids[cluster_id] > 0) {

            processed_detections++;
        }
    }

    /*
       STEP 4:
       Save association results for evaluation.
    */

    save_cluster_associations(
        log_file,
        frame,
        frame_count,
        detection_cluster_ids,
        cluster_track_ids
    );

    /*
       STEP 5:
       Save target states to SQLite.
       One transaction is used for the complete frame.
    */

    if (*count > 0) {

        if (!save_targets_batch_to_database(targets, *count)) {

            printf("Warning: SQLite frame save failed.\n");
        }
    }

    /*
       STEP 6:
       Display frame processing information.
    */

    if (pipeline_verbose) {
        printf(
            "Frame detections: %d | Clusters: %d | "
            "Processed clusters: %d | Processed detections: %d\n",
            frame_count,
            cluster_count,
            processed_clusters,
            processed_detections
        );
    }

    free(clusters);
    free(detection_cluster_ids);
    free(cluster_track_ids);

    return processed_detections;
}

/* ==========================================
   REAL RADAR DATASET PROCESSING
   ========================================== */

void process_real_radar_dataset(
    Target targets[],
    int *count
)
{
    RadarDetection detection;

    RadarDetection *frame = NULL;

    size_t frame_capacity = 0;

    int frame_count = 0;
    int next_track_id = 1;

    long total_detections = 0;
    long processed_detections = 0;
    long rejected_detections = 0;
    long total_frames = 0;

    double current_timestamp = 0.0;

    int frame_started = 0;

    FILE *report = fopen(
        "radar_processing_report.txt",
        "w"
    );

    FILE *log_file = fopen(
        ASSOCIATION_LOG_PATH,
        "w"
    );

    printf("\n========================================\n");
    printf("       REAL RADAR DATASET PROCESSING\n");
    printf("========================================\n");

    printf("Dataset path: %s\n", dataset_path);

    if (report == NULL) {
        printf("Warning: Could not create processing report.\n");
    }

    if (log_file == NULL) {
        printf("Warning: Could not create association log.\n");
    }
    else {
        fprintf(
            log_file,
            "Timestamp,Sensor_ID,Reference_ID,Internal_ID,Label_ID,X,Y\n"
        );
    }

    /* Open the predefined CSV path. */

    if (!open_radar_csv(dataset_path)) {

        printf("\nERROR: Could not open RadarScenes CSV.\n");
        printf("Check whether this file exists:\n");
        printf("%s\n", dataset_path);

        if (report != NULL) {
            fclose(report);
        }

        if (log_file != NULL) {
            fclose(log_file);
        }

        return;
    }

    /* Reset previous target information. */

    memset(
        targets,
        0,
        sizeof(Target) * MAX_TARGETS
    );

    *count = 0;

    tracker_reset_counters();

    /* Start the database from a clean state so results from
       earlier runs do not mix with this run. */
    if (reset_tracking_database()) {
        printf("Previous tracking records cleared from database.\n");
    }
    else {
        printf("Warning: Could not clear previous database records.\n");
    }

    printf("\nCSV file opened successfully.\n");
    printf("Processing RadarScenes sequence_151...\n\n");

    /* Read detections one by one. */

    while (read_radar_detection(&detection)) {

        total_detections++;

        /* Initialize first frame. */

        if (!frame_started) {

            current_timestamp = detection.timestamp;

            frame_started = 1;
        }

        /* Detect timestamp change. */

        else if (fabs(
            detection.timestamp - current_timestamp
        ) > 0.000001) {

            /* Process the previous frame. */

            int result = process_frame_buffer(
                targets,
                count,
                &next_track_id,
                frame,
                frame_count,
                log_file
            );

            processed_detections += result;

            if (frame_count > result) {
                rejected_detections += frame_count - result;
            }

            total_frames++;

            /* Start a new frame. */

            frame_count = 0;

            current_timestamp = detection.timestamp;
        }

        /* Allocate or increase frame buffer. */

        if ((size_t)frame_count >= frame_capacity) {

            size_t new_capacity =
                (frame_capacity == 0)
                ? 256
                : frame_capacity * 2;

            if (new_capacity < frame_capacity ||
                new_capacity > ((size_t)-1) / sizeof(RadarDetection)) {

                printf("Frame buffer capacity overflow.\n");

                rejected_detections++;
                break;
            }

            RadarDetection *new_frame = realloc(
                frame,
                new_capacity * sizeof(RadarDetection)
            );

            if (new_frame == NULL) {

                printf("Memory allocation failed.\n");

                rejected_detections++;
                break;
            }

            frame = new_frame;

            frame_capacity = new_capacity;
        }

        /* Store detection in current frame. */

        frame[frame_count] = detection;

        frame_count++;

        /* Display first few detections. */

        if (total_detections <= 5) {
            display_radar_detection(&detection);
        }

        /* Display processing progress. */

        if (total_detections % 10000 == 0) {

            printf(
                "\nRead: %ld | Processed: %ld | "
                "Frames: %ld | Tracks: %d\n",
                total_detections,
                processed_detections,
                total_frames,
                *count
            );
        }
    }

    /* Process the final frame. */

    if (frame_count > 0) {

        int result = process_frame_buffer(
            targets,
            count,
            &next_track_id,
            frame,
            frame_count,
            log_file
        );

        processed_detections += result;

        if (frame_count > result) {
            rejected_detections += frame_count - result;
        }

        total_frames++;
    }

    /* Release resources. */

    free(frame);

    close_radar_csv();

    if (log_file != NULL) {

        fclose(log_file);

        printf(
            "\nAssociation log saved: %s\n",
            ASSOCIATION_LOG_PATH
        );
    }

    /* Export final target information. */

    export_targets_json(targets, *count);

    /* Display processing summary. */

    printf("\n========================================\n");
    printf("          RADAR PROCESSING SUMMARY\n");
    printf("========================================\n");

    printf(
        "Total detections read: %ld\n",
        total_detections
    );

    printf(
        "Processed detections: %ld\n",
        processed_detections
    );

    printf(
        "Unclustered or unprocessed detections: %ld\n",
        rejected_detections
    );

    printf(
        "Radar frames processed: %ld\n",
        total_frames
    );

    printf(
        "Total tracks created: %d\n",
        next_track_id - 1
    );

    printf(
        "Confirmed tracks: %d\n",
        tracker_tracks_confirmed()
    );

    printf(
        "Track slots in memory (inactive slots are reused): %d\n",
        *count
    );

    printf(
        "Clustering epsilon: %.2f metres\n",
        (double)CLUSTER_EPSILON
    );

    printf(
        "Minimum cluster points: %d\n",
        CLUSTER_MIN_POINTS
    );

    /* Write processing report. */

    if (report != NULL) {

        fprintf(report, "RADAR PROCESSING REPORT\n");
        fprintf(report, "=======================\n");

        fprintf(report, "Dataset: RadarScenes\n");
        fprintf(report, "Sequence: sequence_151\n");

        fprintf(
            report,
            "Clustering epsilon: %.2f metres\n",
            (double)CLUSTER_EPSILON
        );

        fprintf(
            report,
            "Minimum cluster points: %d\n",
            CLUSTER_MIN_POINTS
        );

        fprintf(
            report,
            "Total detections: %ld\n",
            total_detections
        );

        fprintf(
            report,
            "Processed detections: %ld\n",
            processed_detections
        );

        fprintf(
            report,
            "Unclustered or unprocessed detections: %ld\n",
            rejected_detections
        );

        fprintf(
            report,
            "Radar frames: %ld\n",
            total_frames
        );

        fprintf(
            report,
            "Total tracks created: %d\n",
            next_track_id - 1
        );

        fprintf(
            report,
            "Confirmed tracks: %d\n",
            tracker_tracks_confirmed()
        );

        fprintf(
            report,
            "Track slots in memory (inactive slots are reused): %d\n",
            *count
        );

        fclose(report);

        printf("\nReport saved: radar_processing_report.txt\n");
    }

    /* Display final tracked targets. */

    display_targets(targets, *count);

    /* Save final dataset targets to SQLite. */

    save_all_targets_to_database(targets, *count);

    printf("\nRadar processing completed.\n");
}

/* ==========================================
   SAVE TARGETS TO SQLITE DATABASE
   ========================================== */

void save_all_targets_to_database(
    const Target targets[],
    int count
)
{
    if (count <= 0) {
        return;
    }

    if (save_targets_batch_to_database(targets, count)) {

        printf(
            "\nSQLite: saved %d target records successfully.\n",
            count
        );
    }
    else {

        printf("\nSQLite: batch save failed.\n");
    }
}

