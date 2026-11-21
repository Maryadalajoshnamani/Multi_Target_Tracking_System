
#ifndef TRACKER_H
#define TRACKER_H

#include "target.h"
#include "radar_reader.h"
#include "clustering.h"

/* Find an existing target */
int find_target(
    const Target targets[],
    int count,
    int id
);

/* Update a target using a known ID */
int update_target(
    Target targets[],
    int *count,
    int id,
    double x,
    double y,
    double timestamp
);

/* Associate one radar detection */
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
    double gate_distance
);

/* Associate all detections in one radar frame */
int update_radar_frame(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarDetection detections[],
    int detection_count,
    double gate_distance
);

/* Associate cluster centroids with existing tracks */
int update_radar_clusters(
    Target targets[],
    int *count,
    int *next_track_id,
    const RadarCluster clusters[],
    int cluster_count,
    int cluster_track_ids[],
    double gate_distance
);

/* Display target information */
void display_targets(
    const Target targets[],
    int count
);

/* Display target history */
void display_target_history(
    const Target targets[],
    int count,
    int id
);

/* Predict future positions */
void display_prediction(
    const Target targets[],
    int count,
    double future_seconds
);

/* Simulate target movement */
void simulate_target_step(
    Target targets[],
    int *count,
    double time_step
);

/* Display tracking accuracy */
void display_tracking_accuracy(
    const Target targets[],
    int count
);

/* Export target information */
void export_targets_json(
    const Target targets[],
    int count
);

/* Kalman filter functions */
void initialize_kalman(Target *target);

void update_kalman(
    Target *target,
    double measured_x,
    double measured_y,
    double dt
);


/* Track-lifecycle statistics for the real-data pipeline */
void tracker_reset_counters(void);
int tracker_tracks_created(void);
int tracker_tracks_confirmed(void);

#endif
