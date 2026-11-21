#include <stdio.h>
#include <sqlite3.h>

#include "database.h"

static sqlite3 *db = NULL;

static const char *database_path = "database/radar_tracking.db";

void database_set_path(const char *path)
{
    if (path != NULL && path[0] != '\0') {
        database_path = path;
    }
}

/* ==========================================
   CHECK WHETHER A TABLE EXISTS
   ========================================== */

static int table_exists(const char *table_name)
{
    const char *sql =
        "SELECT COUNT(*) FROM sqlite_master "
        "WHERE type = 'table' AND name = ?;";

    sqlite3_stmt *stmt = NULL;

    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK)
        return 0;

    sqlite3_bind_text(stmt, 1, table_name, -1, SQLITE_TRANSIENT);

    int exists = 0;

    if (sqlite3_step(stmt) == SQLITE_ROW)
        exists = sqlite3_column_int(stmt, 0) > 0;

    sqlite3_finalize(stmt);

    return exists;
}

/* ==========================================
   BIND TARGET DATA
   ========================================== */

static void bind_target(
    sqlite3_stmt *stmt,
    const Target *target
)
{
    sqlite3_bind_int(stmt, 1, target->id);
    sqlite3_bind_int(stmt, 2, target->sensor_id);

    sqlite3_bind_double(stmt, 3, target->x);
    sqlite3_bind_double(stmt, 4, target->y);

    sqlite3_bind_double(stmt, 5, target->vx);
    sqlite3_bind_double(stmt, 6, target->vy);
    sqlite3_bind_double(stmt, 7, target->speed);

    sqlite3_bind_double(stmt, 8, target->range);
    sqlite3_bind_double(stmt, 9, target->azimuth);
    sqlite3_bind_double(stmt, 10, target->rcs);
    sqlite3_bind_double(stmt, 11, target->radial_velocity);

    sqlite3_bind_double(stmt, 12, target->timestamp);

    sqlite3_bind_int(stmt, 13, target->active);
    sqlite3_bind_int(stmt, 14, target->missed_frames);
    sqlite3_bind_int(stmt, 15, target->detection_count);

    sqlite3_bind_text(
        stmt, 16, target->uuid, -1, SQLITE_TRANSIENT
    );

    sqlite3_bind_text(
        stmt, 17, target->reference_track_id,
        -1, SQLITE_TRANSIENT
    );

    sqlite3_bind_int(stmt, 18, target->label_id);
}

/* ==========================================
   INITIALIZE DATABASE
   ========================================== */

int initialize_database(void)
{
    int rc;

    const char *create_latest_table =
        "CREATE TABLE IF NOT EXISTS radar_target_latest ("
        "target_id INTEGER PRIMARY KEY,"
        "sensor_id INTEGER,"
        "x REAL,"
        "y REAL,"
        "vx REAL,"
        "vy REAL,"
        "speed REAL,"
        "range REAL,"
        "azimuth REAL,"
        "rcs REAL,"
        "radial_velocity REAL,"
        "timestamp REAL,"
        "active INTEGER,"
        "missed_frames INTEGER,"
        "detection_count INTEGER,"
        "uuid TEXT,"
        "reference_track_id TEXT,"
        "label_id INTEGER,"
        "last_saved_at TEXT DEFAULT CURRENT_TIMESTAMP"
        ");";

    const char *create_history_table =
        "CREATE TABLE IF NOT EXISTS radar_target_history ("
        "history_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "target_id INTEGER NOT NULL,"
        "sensor_id INTEGER,"
        "x REAL,"
        "y REAL,"
        "vx REAL,"
        "vy REAL,"
        "speed REAL,"
        "range REAL,"
        "azimuth REAL,"
        "rcs REAL,"
        "radial_velocity REAL,"
        "timestamp REAL,"
        "active INTEGER,"
        "missed_frames INTEGER,"
        "detection_count INTEGER,"
        "uuid TEXT,"
        "reference_track_id TEXT,"
        "label_id INTEGER,"
        "recorded_at TEXT DEFAULT CURRENT_TIMESTAMP,"
        "legacy_record_id INTEGER UNIQUE"
        ");";

    rc = sqlite3_open(
        database_path,
        &db
    );

    if (rc != SQLITE_OK)
    {
        fprintf(
            stderr,
            "SQLite open failed: %s\n",
            db ? sqlite3_errmsg(db) : "Unknown error"
        );

        if (db != NULL)
        {
            sqlite3_close(db);
            db = NULL;
        }

        return 0;
    }

    rc = sqlite3_exec(
        db,
        create_latest_table,
        NULL,
        NULL,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "Latest table creation failed: %s\n",
                sqlite3_errmsg(db));

        close_database();
        return 0;
    }

    rc = sqlite3_exec(
        db,
        create_history_table,
        NULL,
        NULL,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "History table creation failed: %s\n",
                sqlite3_errmsg(db));

        close_database();
        return 0;
    }

    /* Speeds up the duplicate check used when saving history. */
    rc = sqlite3_exec(
        db,
        "CREATE INDEX IF NOT EXISTS idx_history_target_time "
        "ON radar_target_history(target_id, timestamp);",
        NULL,
        NULL,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "Index creation failed: %s\n",
                sqlite3_errmsg(db));

        close_database();
        return 0;
    }

    /* ======================================
       MIGRATE OLD DATABASE RECORDS
       ====================================== */

    if (table_exists("radar_targets"))
    {
        const char *migrate_history =
            "INSERT OR IGNORE INTO radar_target_history ("
            "target_id, sensor_id, x, y, vx, vy, speed, "
            "range, azimuth, rcs, radial_velocity, timestamp, "
            "active, missed_frames, detection_count, uuid, "
            "reference_track_id, label_id, legacy_record_id"
            ") "
            "SELECT "
            "target_id, sensor_id, x, y, vx, vy, speed, "
            "range, azimuth, rcs, radial_velocity, timestamp, "
            "active, missed_frames, detection_count, uuid, "
            "reference_track_id, label_id, record_id "
            "FROM radar_targets;";

        rc = sqlite3_exec(
            db,
            migrate_history,
            NULL,
            NULL,
            NULL
        );

        if (rc != SQLITE_OK)
        {
            fprintf(stderr, "History migration failed: %s\n",
                    sqlite3_errmsg(db));

            close_database();
            return 0;
        }

        const char *migrate_latest =
            "INSERT OR IGNORE INTO radar_target_latest ("
            "target_id, sensor_id, x, y, vx, vy, speed, "
            "range, azimuth, rcs, radial_velocity, timestamp, "
            "active, missed_frames, detection_count, uuid, "
            "reference_track_id, label_id"
            ") "
            "SELECT "
            "target_id, sensor_id, x, y, vx, vy, speed, "
            "range, azimuth, rcs, radial_velocity, timestamp, "
            "active, missed_frames, detection_count, uuid, "
            "reference_track_id, label_id "
            "FROM radar_targets "
            "WHERE record_id IN ("
            "SELECT MAX(record_id) FROM radar_targets "
            "GROUP BY target_id"
            ");";

        rc = sqlite3_exec(
            db,
            migrate_latest,
            NULL,
            NULL,
            NULL
        );

        if (rc != SQLITE_OK)
        {
            fprintf(stderr, "Latest migration failed: %s\n",
                    sqlite3_errmsg(db));

            close_database();
            return 0;
        }
    }

    printf("\nSQLite database initialized successfully.\n");
    printf("Database: %s\n", database_path);
    printf("Latest target table: radar_target_latest\n");
    printf("History table: radar_target_history\n");

    return 1;
}

/* ==========================================
   BATCH SAVE TARGETS IN ONE TRANSACTION
   ========================================== */

int save_targets_batch_to_database(
    const Target targets[],
    int count
)
{
    if (db == NULL || targets == NULL || count < 0)
    {
        fprintf(stderr, "SQLite batch save failed: invalid input.\n");
        return 0;
    }

    if (count == 0)
        return 1;

    const char *history_sql =
        "INSERT INTO radar_target_history ("
        "target_id, sensor_id, x, y, vx, vy, speed, "
        "range, azimuth, rcs, radial_velocity, timestamp, "
        "active, missed_frames, detection_count, uuid, "
        "reference_track_id, label_id"
        ") "
        "SELECT ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ? "
        "WHERE NOT EXISTS ("
        "SELECT 1 FROM radar_target_history "
        "WHERE target_id = ? "
        "AND timestamp = ? "
        "AND x = ? "
        "AND y = ?"
        ");";

    const char *latest_sql =
        "INSERT INTO radar_target_latest ("
        "target_id, sensor_id, x, y, vx, vy, speed, "
        "range, azimuth, rcs, radial_velocity, timestamp, "
        "active, missed_frames, detection_count, uuid, "
        "reference_track_id, label_id"
        ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) "

        "ON CONFLICT(target_id) DO UPDATE SET "
        "sensor_id = excluded.sensor_id, "
        "x = excluded.x, "
        "y = excluded.y, "
        "vx = excluded.vx, "
        "vy = excluded.vy, "
        "speed = excluded.speed, "
        "range = excluded.range, "
        "azimuth = excluded.azimuth, "
        "rcs = excluded.rcs, "
        "radial_velocity = excluded.radial_velocity, "
        "timestamp = excluded.timestamp, "
        "active = excluded.active, "
        "missed_frames = excluded.missed_frames, "
        "detection_count = excluded.detection_count, "
        "uuid = excluded.uuid, "
        "reference_track_id = excluded.reference_track_id, "
        "label_id = excluded.label_id, "
        "last_saved_at = CURRENT_TIMESTAMP "
        "WHERE excluded.timestamp >= radar_target_latest.timestamp;";

    sqlite3_stmt *history_stmt = NULL;
    sqlite3_stmt *latest_stmt = NULL;

    int rc;

    rc = sqlite3_exec(
        db,
        "BEGIN TRANSACTION;",
        NULL,
        NULL,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "SQLite transaction failed: %s\n",
                sqlite3_errmsg(db));

        return 0;
    }

    rc = sqlite3_prepare_v2(
        db,
        history_sql,
        -1,
        &history_stmt,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "History prepare failed: %s\n",
                sqlite3_errmsg(db));

        goto rollback;
    }

    rc = sqlite3_prepare_v2(
        db,
        latest_sql,
        -1,
        &latest_stmt,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "Latest prepare failed: %s\n",
                sqlite3_errmsg(db));

        goto rollback;
    }

    for (int i = 0; i < count; i++)
    {
        const Target *target = &targets[i];

        /* Tentative tracks are not stored until they are confirmed. */
        if (!target->confirmed) {
            continue;
        }

        /* Save historical observation */

        sqlite3_reset(history_stmt);
        sqlite3_clear_bindings(history_stmt);

        bind_target(history_stmt, target);

        sqlite3_bind_int(history_stmt, 19, target->id);
        sqlite3_bind_double(history_stmt, 20, target->timestamp);
        sqlite3_bind_double(history_stmt, 21, target->x);
        sqlite3_bind_double(history_stmt, 22, target->y);

        rc = sqlite3_step(history_stmt);

        if (rc != SQLITE_DONE)
        {
            fprintf(stderr,
                    "History insert failed for target %d: %s\n",
                    target->id,
                    sqlite3_errmsg(db));

            goto rollback;
        }

        /* Update latest target state */

        sqlite3_reset(latest_stmt);
        sqlite3_clear_bindings(latest_stmt);

        bind_target(latest_stmt, target);

        rc = sqlite3_step(latest_stmt);

        if (rc != SQLITE_DONE)
        {
            fprintf(stderr,
                    "Latest target update failed for target %d: %s\n",
                    target->id,
                    sqlite3_errmsg(db));

            goto rollback;
        }
    }

    sqlite3_finalize(history_stmt);
    sqlite3_finalize(latest_stmt);

    history_stmt = NULL;
    latest_stmt = NULL;

    rc = sqlite3_exec(
        db,
        "COMMIT;",
        NULL,
        NULL,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "SQLite commit failed: %s\n",
                sqlite3_errmsg(db));

        sqlite3_exec(db, "ROLLBACK;", NULL, NULL, NULL);

        return 0;
    }

    return 1;

rollback:

    if (history_stmt != NULL)
        sqlite3_finalize(history_stmt);

    if (latest_stmt != NULL)
        sqlite3_finalize(latest_stmt);

    sqlite3_exec(db, "ROLLBACK;", NULL, NULL, NULL);

    return 0;
}

/* ==========================================
   SAVE ONE TARGET
   ========================================== */

int save_target_to_database(const Target *target)
{
    if (target == NULL)
    {
        fprintf(stderr, "SQLite save failed: target is NULL.\n");
        return 0;
    }

    return save_targets_batch_to_database(target, 1);
}

/* ==========================================
   CLEAR TRACKING TABLES BEFORE A NEW RUN
   ========================================== */

int reset_tracking_database(void)
{
    if (db == NULL)
    {
        return 0;
    }

    char *error_message = NULL;

    int rc = sqlite3_exec(
        db,
        "BEGIN TRANSACTION;"
        "DELETE FROM radar_target_history;"
        "DELETE FROM radar_target_latest;"
        "DELETE FROM sqlite_sequence "
        "WHERE name = 'radar_target_history';"
        "COMMIT;",
        NULL,
        NULL,
        &error_message
    );

    if (rc != SQLITE_OK)
    {
        fprintf(stderr, "Database reset failed: %s\n",
                error_message ? error_message : "unknown error");

        sqlite3_free(error_message);
        sqlite3_exec(db, "ROLLBACK;", NULL, NULL, NULL);

        return 0;
    }

    return 1;
}

/* ==========================================
   DISPLAY TARGET HISTORY
   ========================================== */

void display_database_target_history(int target_id)
{
    if (db == NULL)
    {
        printf("Database is not initialized.\n");
        return;
    }

    const char *sql =
        "SELECT history_id, x, y, vx, vy, speed, "
        "timestamp, active, detection_count "
        "FROM radar_target_history "
        "WHERE target_id = ? "
        "ORDER BY timestamp ASC, history_id ASC;";

    sqlite3_stmt *stmt = NULL;

    int rc = sqlite3_prepare_v2(
        db,
        sql,
        -1,
        &stmt,
        NULL
    );

    if (rc != SQLITE_OK)
    {
        printf("History query failed: %s\n", sqlite3_errmsg(db));
        return;
    }

    sqlite3_bind_int(stmt, 1, target_id);

    printf("\n========================================\n");
    printf("       DATABASE TARGET HISTORY\n");
    printf("========================================\n");

    printf("Target ID: %d\n", target_id);

    int records = 0;

    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW)
    {
        records++;

        printf("\nHistory Record: %d\n", records);

        printf("History ID: %d\n",
               sqlite3_column_int(stmt, 0));

        printf("Position: (%.2f, %.2f)\n",
               sqlite3_column_double(stmt, 1),
               sqlite3_column_double(stmt, 2));

        printf("Velocity: (%.2f, %.2f)\n",
               sqlite3_column_double(stmt, 3),
               sqlite3_column_double(stmt, 4));

        printf("Speed: %.2f\n",
               sqlite3_column_double(stmt, 5));

        printf("Timestamp: %.6f\n",
               sqlite3_column_double(stmt, 6));

        printf("Status: %s\n",
               sqlite3_column_int(stmt, 7) ? "Active" : "Inactive");

        printf("Detection Count: %d\n",
               sqlite3_column_int(stmt, 8));
    }

    if (rc != SQLITE_DONE)
    {
        printf("History reading failed: %s\n",
               sqlite3_errmsg(db));
    }
    else if (records == 0)
    {
        printf("No history records found for this target.\n");
    }
    else
    {
        printf("\nTotal historical records: %d\n", records);
    }

    sqlite3_finalize(stmt);
}

/* ==========================================
   CLOSE DATABASE
   ========================================== */

void close_database(void)
{
    if (db != NULL)
    {
        sqlite3_close(db);
        db = NULL;

        printf("\nSQLite database connection closed.\n");
    }
}