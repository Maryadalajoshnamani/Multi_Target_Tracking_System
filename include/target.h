
#ifndef TARGET_H
#define TARGET_H

#define MAX_TARGETS 10000
#define MAX_HISTORY 100

#define MAX_UUID_LENGTH 40
#define MAX_REFERENCE_ID_LENGTH 40

typedef struct {
    double x;
    double y;
    double timestamp;
} TrackPoint;

typedef struct {
    double state[4];
    double covariance[4][4];
    int initialized;
} KalmanState;

typedef struct {
    int id;

    int sensor_id;
    double range;
    double azimuth;
    double rcs;
    double radial_velocity;

    char uuid[MAX_UUID_LENGTH];
    char reference_track_id[MAX_REFERENCE_ID_LENGTH];
    int label_id;

    double x;
    double y;
    double timestamp;

    double vx;
    double vy;
    double speed;

    int active;
    int confirmed;   /* 0 = tentative track, 1 = confirmed */
    int missed_frames;
    int detection_count;

    double prediction_error;
    double total_prediction_error;
    int prediction_count;
    int has_prediction_error;

    KalmanState kalman;

    double average_raw_measurement_error;
    double average_kalman_estimation_error;
    double error_reduction_percent;

    double total_raw_measurement_error;
    double total_kalman_estimation_error;

    int noise_comparison_count;

    TrackPoint history[MAX_HISTORY];
    int history_count;

} Target;

#endif
