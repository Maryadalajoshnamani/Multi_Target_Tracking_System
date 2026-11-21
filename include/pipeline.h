#ifndef PIPELINE_H
#define PIPELINE_H

#include "target.h"

/* Choose the RadarScenes radar_detections.csv file to process. */
void pipeline_set_dataset_path(const char *path);

/* 1 = print one line per radar frame while processing. */
void pipeline_set_verbose(int verbose);

/*
 * Run the complete real-data pipeline:
 * CSV reading -> clustering -> track association -> Kalman filter
 * -> SQLite storage -> reports and JSON export.
 */
void process_real_radar_dataset(
    Target targets[],
    int *count
);

/* Save all confirmed targets to the SQLite database. */
void save_all_targets_to_database(
    const Target targets[],
    int count
);

#endif
