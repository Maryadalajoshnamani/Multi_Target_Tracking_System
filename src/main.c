
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "tracker.h"
#include "radar_reader.h"
#include "clustering.h"
#include "database.h"
#include "pipeline.h"

#define MAX_SIMULATION_STEPS 1000

/* ==========================================
   WAIT FUNCTION
   ========================================== */

void wait_one_second(void)
{
#ifdef _WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
}

/* ==========================================
   LOAD DEMO DATA
   ========================================== */

void load_demo_data(Target targets[], int *count)
{
    struct Observation {
        int id;
        double x;
        double y;
        double time;
    };

    struct Observation data[] = {
        {101, 10, 20, 0},
        {202, 50, 15, 0},
        {303, 25, 60, 0},
        {101, 12, 22, 1},
        {202, 49, 17, 1},
        {303, 27, 59, 1},
        {101, 14, 24, 2},
        {202, 48, 19, 2},
        {303, 29, 58, 2}
    };

    int total = sizeof(data) / sizeof(data[0]);

    for (int i = 0; i < total; i++) {
        update_target(
            targets,
            count,
            data[i].id,
            data[i].x,
            data[i].y,
            data[i].time
        );
    }
}

/* ==========================================
   AUTOMATIC SIMULATION
   ========================================== */

void run_automatic_simulation(
    Target targets[],
    int *count
)
{
    int steps;

    printf("\nEnter number of simulation steps: ");

    if (scanf("%d", &steps) != 1 ||
        steps < 1 ||
        steps > MAX_SIMULATION_STEPS) {

        printf("Invalid number of steps.\n");
        return;
    }

    for (int i = 1; i <= steps; i++) {

        printf(
            "\n========== SIMULATION STEP %d ==========\n",
            i
        );

        simulate_target_step(targets, count, 1.0);

        display_targets(targets, *count);

        display_tracking_accuracy(targets, *count);

        export_targets_json(targets, *count);

        if (i < steps) {
            wait_one_second();
        }
    }

    printf("\nSimulation completed.\n");
}

/* ==========================================
   MEASUREMENT NOISE
   ========================================== */

double generate_measurement_noise(void)
{
    return ((double)rand() / RAND_MAX) * 4.0 - 2.0;
}

/* ==========================================
   KALMAN NOISE SIMULATION
   ========================================== */

void run_noise_simulation(
    Target targets[],
    int *count
)
{
    int steps;

    printf("\nEnter simulation steps (1-1000): ");

    if (scanf("%d", &steps) != 1 ||
        steps < 1 ||
        steps > MAX_SIMULATION_STEPS) {

        printf("Invalid input.\n");
        return;
    }

    if (*count == 0) {
        printf("No targets available.\n");
        return;
    }

    double true_x[MAX_TARGETS];
    double true_y[MAX_TARGETS];
    double true_vx[MAX_TARGETS];
    double true_vy[MAX_TARGETS];
    double timestamps[MAX_TARGETS];

    int original_count = *count;

    for (int i = 0; i < original_count; i++) {

        true_x[i] = targets[i].x;
        true_y[i] = targets[i].y;

        true_vx[i] = targets[i].vx;
        true_vy[i] = targets[i].vy;

        timestamps[i] = targets[i].timestamp;

        targets[i].total_raw_measurement_error = 0.0;
        targets[i].total_kalman_estimation_error = 0.0;

        targets[i].noise_comparison_count = 0;
    }

    for (int step = 1; step <= steps; step++) {

        printf(
            "\n========== NOISE STEP %d ==========\n",
            step
        );

        for (int i = 0; i < original_count; i++) {

            int id = targets[i].id;

            true_x[i] += true_vx[i];
            true_y[i] += true_vy[i];

            double measured_x =
                true_x[i] + generate_measurement_noise();

            double measured_y =
                true_y[i] + generate_measurement_noise();

            timestamps[i] += 1.0;

            update_target(
                targets,
                count,
                id,
                measured_x,
                measured_y,
                timestamps[i]
            );

            double raw_dx = measured_x - true_x[i];
            double raw_dy = measured_y - true_y[i];

            double raw_error = sqrt(
                raw_dx * raw_dx + raw_dy * raw_dy
            );

            int index = find_target(targets, *count, id);

            if (index < 0) {
                continue;
            }

            double filtered_dx =
                targets[index].kalman.state[0] - true_x[i];

            double filtered_dy =
                targets[index].kalman.state[1] - true_y[i];

            double kalman_error = sqrt(
                filtered_dx * filtered_dx +
                filtered_dy * filtered_dy
            );

            targets[index].total_raw_measurement_error += raw_error;

            targets[index].total_kalman_estimation_error +=
                kalman_error;

            targets[index].noise_comparison_count++;

            int samples =
                targets[index].noise_comparison_count;

            targets[index].average_raw_measurement_error =
                targets[index].total_raw_measurement_error / samples;

            targets[index].average_kalman_estimation_error =
                targets[index].total_kalman_estimation_error / samples;

            if (targets[index].average_raw_measurement_error > 0.000001) {

                targets[index].error_reduction_percent =
                    (
                        targets[index].average_raw_measurement_error -
                        targets[index].average_kalman_estimation_error
                    ) /
                    targets[index].average_raw_measurement_error * 100.0;
            }

            printf(
                "Target %d | Raw Error: %.2f | Kalman Error: %.2f\n",
                id,
                raw_error,
                kalman_error
            );
        }

        export_targets_json(targets, *count);

        if (step < steps) {
            wait_one_second();
        }
    }

    printf("\nNoise simulation completed.\n");
}

/* ==========================================
   MAIN FUNCTION
   ========================================== */

static void print_usage(const char *program)
{
    printf("Usage: %s [options]\n\n", program);
    printf("Without options the interactive menu is shown.\n\n");
    printf("Options:\n");
    printf("  --process        process the RadarScenes dataset and exit\n");
    printf("  --data <file>    radar_detections.csv to process\n");
    printf("  --db <file>      SQLite database file\n");
    printf("  --verbose        print one line per radar frame\n");
    printf("  --help           show this help\n");
}

int main(int argc, char **argv)
{
    int process_only = 0;

    for (int i = 1; i < argc; i++) {

        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        else if (strcmp(argv[i], "--process") == 0) {
            process_only = 1;
        }
        else if (strcmp(argv[i], "--verbose") == 0) {
            pipeline_set_verbose(1);
        }
        else if (strcmp(argv[i], "--data") == 0 && i + 1 < argc) {
            pipeline_set_dataset_path(argv[++i]);
        }
        else if (strcmp(argv[i], "--db") == 0 && i + 1 < argc) {
            database_set_path(argv[++i]);
        }
        else {
            printf("Unknown or incomplete option: %s\n\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    /*
       Static allocation prevents the large
       target array from consuming the stack.
    */

    static Target targets[MAX_TARGETS];

    int count = 0;
    int choice;

    srand((unsigned int)time(NULL));

    memset(targets, 0, sizeof(targets));

    /* Initialize SQLite database. */

    if (!initialize_database()) {

        printf("SQLite database initialization failed.\n");

        return 1;
    }

    /* Headless mode: process the dataset and exit. */

    if (process_only) {

        process_real_radar_dataset(targets, &count);

        close_database();

        return 0;
    }

    /* Load demo targets. */

    load_demo_data(targets, &count);

    export_targets_json(targets, count);

    /* Save initial demo targets to SQLite. */

    save_all_targets_to_database(targets, count);

    printf("\n========================================\n");
    printf("       MULTI-TARGET TRACKING SYSTEM\n");
    printf("========================================\n");

    while (1) {

        printf("\n------------- MAIN MENU -------------\n");

        printf("1. Display all targets\n");
        printf("2. Add or update target\n");
        printf("3. View target history\n");
        printf("4. Predict future positions\n");
        printf("5. Exit\n");
        printf("6. Start automatic simulation\n");
        printf("7. View tracking accuracy report\n");
        printf("8. Run measurement noise simulation\n");
        printf("9. Process Real RadarScenes Dataset\n");

        printf("\nEnter your choice: ");

        if (scanf("%d", &choice) != 1) {

            printf("Invalid input. Exiting.\n");

            close_database();

            return 1;
        }

        switch (choice) {

            case 1:

                display_targets(targets, count);

                break;

            case 2: {

                int id;

                double x, y, timestamp;

                printf("Enter target ID: ");

                if (scanf("%d", &id) != 1) {
                    printf("Invalid target ID.\n");
                    break;
                }

                printf("Enter X position: ");

                if (scanf("%lf", &x) != 1) {
                    printf("Invalid X position.\n");
                    break;
                }

                printf("Enter Y position: ");

                if (scanf("%lf", &y) != 1) {
                    printf("Invalid Y position.\n");
                    break;
                }

                printf("Enter timestamp: ");

                if (scanf("%lf", &timestamp) != 1) {
                    printf("Invalid timestamp.\n");
                    break;
                }

                if (update_target(
                    targets,
                    &count,
                    id,
                    x,
                    y,
                    timestamp
                )) {

                    printf("Target updated successfully.\n");

                    export_targets_json(targets, count);

                    save_all_targets_to_database(targets, count);

                } else {

                    printf("Target update failed.\n");
                }

                break;
            }

            case 3: {

                int id;

                printf("Enter target ID: ");

                if (scanf("%d", &id) != 1) {
                    printf("Invalid target ID.\n");
                    break;
                }

                display_database_target_history(id);

                break;
            }

            case 4: {

                double future_seconds;

                printf("Enter future time interval: ");

                if (scanf("%lf", &future_seconds) != 1) {
                    printf("Invalid time interval.\n");
                    break;
                }

                if (future_seconds >= 0) {

                    display_prediction(
                        targets,
                        count,
                        future_seconds
                    );

                } else {

                    printf("Enter a non-negative interval.\n");
                }

                break;
            }

            case 5:

                export_targets_json(targets, count);

                printf("Exiting application.\n");

                close_database();

                return 0;

            case 6:

                run_automatic_simulation(
                    targets,
                    &count
                );

                save_all_targets_to_database(targets, count);

                break;

            case 7:

                display_tracking_accuracy(
                    targets,
                    count
                );

                break;

            case 8:

                run_noise_simulation(
                    targets,
                    &count
                );

                save_all_targets_to_database(targets, count);

                break;

            case 9:

                process_real_radar_dataset(
                    targets,
                    &count
                );

                break;

            default:

                printf("Invalid choice. Try again.\n");

                break;
        }
    }

    return 0;
}
