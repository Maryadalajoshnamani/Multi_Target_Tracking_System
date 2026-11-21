
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "clustering.h"

/* Check whether two detections belong to the same radar frame. */
static int same_frame(
    const RadarDetection *a,
    const RadarDetection *b
) {
    if (a->sensor_id != b->sensor_id) {
        return 0;
    }

    return fabs(a->timestamp - b->timestamp) <= 0.000001;
}

/* Calculate distance between two radar detections. */
static double detection_distance(
    const RadarDetection *a,
    const RadarDetection *b
) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;

    return sqrt(dx * dx + dy * dy);
}

/* Find neighbouring detections within the specified radius. */
static int find_neighbors(
    const RadarDetection detections[],
    int detection_count,
    int point_index,
    int neighbors[],
    double epsilon
) {
    int neighbor_count = 0;

    for (int i = 0; i < detection_count; i++) {

        if (!same_frame(&detections[point_index], &detections[i])) {
            continue;
        }

        if (
            detection_distance(
                &detections[point_index],
                &detections[i]
            ) <= epsilon
        ) {
            neighbors[neighbor_count++] = i;
        }
    }

    return neighbor_count;
}

/* Calculate the properties of one cluster. */
static void calculate_cluster_properties(
    const RadarDetection detections[],
    const int detection_cluster_ids[],
    int detection_count,
    int cluster_id,
    RadarCluster *cluster
) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_range = 0.0;
    double sum_azimuth = 0.0;
    double sum_rcs = 0.0;
    double sum_velocity = 0.0;

    int count = 0;

    char reference_id[MAX_REFERENCE_ID_LENGTH] = "";
    int reference_conflict = 0;

    memset(cluster, 0, sizeof(RadarCluster));

    for (int i = 0; i < detection_count; i++) {

        if (detection_cluster_ids[i] != cluster_id) {
            continue;
        }

        const RadarDetection *d = &detections[i];

        sum_x += d->x;
        sum_y += d->y;
        sum_range += d->range;
        sum_azimuth += d->azimuth;
        sum_rcs += d->rcs;
        sum_velocity += d->compensated_velocity;

        cluster->timestamp = d->timestamp;
        cluster->sensor_id = d->sensor_id;

        /*
         * Collect reference track IDs.
         * Ignore empty IDs.
         */
        if (d->reference_track_id[0] != '\0') {

            if (reference_id[0] == '\0') {

                snprintf(reference_id, sizeof(reference_id), "%s", d->reference_track_id);
            }
            else if (
                strcmp(reference_id, d->reference_track_id) != 0
            ) {
                reference_conflict = 1;
            }
        }

        count++;
    }

    if (count == 0) {
        return;
    }

    /* Calculate centroid and average measurements. */
    cluster->x = sum_x / count;
    cluster->y = sum_y / count;
    cluster->range = sum_range / count;
    cluster->azimuth = sum_azimuth / count;
    cluster->rcs = sum_rcs / count;
    cluster->radial_velocity = sum_velocity / count;

    cluster->detection_count = count;

    /*
     * Assign a reference ID only when all non-empty
     * reference IDs in this cluster agree.
     */
    if (!reference_conflict && reference_id[0] != '\0') {

        snprintf(cluster->reference_track_id, sizeof(cluster->reference_track_id), "%s", reference_id);
    }
}

/*
 * Main DBSCAN-style clustering function.
 *
 * Returns the number of clusters generated.
 */
int cluster_radar_detections(
    const RadarDetection detections[],
    int detection_count,
    RadarCluster clusters[],
    int max_clusters,
    int detection_cluster_ids[],
    double epsilon,
    int min_points
) {
    if (
        detections == NULL ||
        clusters == NULL ||
        detection_cluster_ids == NULL ||
        detection_count <= 0 ||
        max_clusters <= 0 ||
        epsilon <= 0.0 ||
        min_points <= 0
    ) {
        return 0;
    }

    int *visited = calloc((size_t)detection_count, sizeof(int));
    int *neighbors = malloc((size_t)detection_count * sizeof(int));
    int *queue = malloc((size_t)detection_count * sizeof(int));
    int *queued = calloc((size_t)detection_count, sizeof(int));

    if (
        visited == NULL ||
        neighbors == NULL ||
        queue == NULL ||
        queued == NULL
    ) {
        fprintf(stderr, "Clustering memory allocation failed.\n");

        free(visited);
        free(neighbors);
        free(queue);
        free(queued);

        return 0;
    }

    /* Initially, all detections are unassigned. */
    for (int i = 0; i < detection_count; i++) {
        detection_cluster_ids[i] = -1;
    }

    int cluster_count = 0;

    for (int i = 0; i < detection_count; i++) {

        if (visited[i]) {
            continue;
        }

        visited[i] = 1;

        int neighbor_count = find_neighbors(
            detections,
            detection_count,
            i,
            neighbors,
            epsilon
        );

        /* Not enough neighbours: this is a noise point for now. */
        if (neighbor_count < min_points) {
            continue;
        }

        if (cluster_count >= max_clusters) {
            break;
        }

        int current_cluster = cluster_count++;

        detection_cluster_ids[i] = current_cluster;

        int queue_start = 0;
        int queue_end = 0;

        memset(
            queued,
            0,
            (size_t)detection_count * sizeof(int)
        );

        /* Add initial neighbours to the expansion queue. */
        for (int j = 0; j < neighbor_count; j++) {

            int index = neighbors[j];

            if (!queued[index]) {
                queue[queue_end++] = index;
                queued[index] = 1;
            }
        }

        /* Expand the cluster. */
        while (queue_start < queue_end) {

            int current_index = queue[queue_start++];

            if (!visited[current_index]) {

                visited[current_index] = 1;

                int current_neighbor_count = find_neighbors(
                    detections,
                    detection_count,
                    current_index,
                    neighbors,
                    epsilon
                );

                if (current_neighbor_count >= min_points) {

                    for (
                        int j = 0;
                        j < current_neighbor_count;
                        j++
                    ) {
                        int next_index = neighbors[j];

                        if (!queued[next_index]) {
                            queue[queue_end++] = next_index;
                            queued[next_index] = 1;
                        }
                    }
                }
            }

            /* Assign previously unassigned points. */
            if (detection_cluster_ids[current_index] == -1) {
                detection_cluster_ids[current_index] = current_cluster;
            }
        }
    }

    /* Calculate the final properties of each cluster. */
    for (int i = 0; i < cluster_count; i++) {

        calculate_cluster_properties(
            detections,
            detection_cluster_ids,
            detection_count,
            i,
            &clusters[i]
        );
    }

    free(visited);
    free(neighbors);
    free(queue);
    free(queued);

    return cluster_count;
}
