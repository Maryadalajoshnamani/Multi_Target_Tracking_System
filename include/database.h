#ifndef DATABASE_H
#define DATABASE_H

#include "target.h"

/* Choose the database file (call before initialize_database) */
void database_set_path(const char *path);

int initialize_database(void);

int save_target_to_database(const Target *target);

int save_targets_batch_to_database(
    const Target targets[],
    int count
);

void display_database_target_history(int target_id);

/* Clear latest and history tables before a new dataset run */
int reset_tracking_database(void);

void close_database(void);

#endif