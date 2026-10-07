/**
 * @file arbor_bridge.h
 * Project: Arbor
 *
 * Copyright (C) 2026 Jean-Romain Roussel (r-lidar) <info @ r-lidar.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ARBOR_BRIDGE_H
#define ARBOR_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Borrowed input columns. A null pointer means that column is absent.
   Coordinates are double here and float inside the engine. The buffers are
   not const so Swift can pass the pointers it allocated; the engine only reads them. */
typedef struct ArborBridgeCloud {
    size_t count;
    double *x;
    double *y;
    double *z;
    int32_t *classification;
    double *hag;
    float *pwood;
    int32_t *foliage;
    int32_t *passage;
    int32_t *user_data;
    int32_t *tree_id;
    uint16_t *red;
    uint16_t *green;
    uint16_t *blue;
} ArborBridgeCloud;

/* Columns the call wrote, in the post-call point order.
   order[new] = old when reordered is non-zero. Pointers are malloc'd. */
typedef struct ArborBridgeAttributes {
    size_t count;
    int reordered;
    int32_t *order;
    int classification_set;
    int32_t *classification;
    int hag_set;
    double *hag;
    int foliage_set;
    int32_t *foliage;
    int passage_set;
    int32_t *passage;
    int user_data_set;
    int32_t *user_data;
    int tree_id_set;
    int32_t *tree_id;
    int rgb_set;
    uint16_t *red;
    uint16_t *green;
    uint16_t *blue;
} ArborBridgeAttributes;

/* New cloud from find_seeds. Pointers are malloc'd. A set flag with a null
   pointer and count 0 is an empty column. */
typedef struct ArborBridgeSeedCloud {
    size_t count;
    double *x;
    double *y;
    double *z;
    int tree_id_set;
    int32_t *tree_id;
    int foliage_set;
    int32_t *foliage;
    int hag_set;
    double *hag;
    int pwood_set;
    float *pwood;
    int passage_set;
    int32_t *passage;
} ArborBridgeSeedCloud;

typedef struct ArborBridgeParams {
    double cut_above_ground;
    int32_t wood_k;
    int downward;
    int32_t graph_k;
    int32_t k_seed;
    double decimation;
    double space_res;
    double max_gap;
    double power;
    double wood_to_wood;
    double leaf_to_leaf;
    double wood_to_leaf;
    const float *angle_penalty;
    int32_t angle_penalty_count;
    int32_t min_passage;
    double high_pwood_threshold;
    double medium_pwood_threshold;
    double connected_components_res;
    int32_t connected_components_min;
    int32_t wood_assignation_k;
    double wood_assignation_dist;
    int32_t wood_extra_reasignation_k;
    double wood_extra_reasignation_dist;
    int32_t medium_pwood_sor_k;
    double medium_pwood_sor_m;
    double ground_res;
    const double *slice_at;
    int32_t slice_at_count;
    double slice_thickness;
    int32_t seed_min_passage;
    double safe_zone;
    int oversegmentation_solver_enabled;
    float skeleton_node_distance;
    float dbscan_eps_distance;
    float max_d;
    float apex_radius;
    int32_t smooth_steps;
    float min_measurable_dbh;
    float min_measurable_radius;
    int broken_detection_enabled;
    float allometry_scale;
    const char *allometry_name;
} ArborBridgeParams;

/* cyl_id is the edge target node and parent_id is the edge source node,
   matching the R data frame written from a QSM graph. */
typedef struct ArborBridgeCylinder {
    double start_x;
    double start_y;
    double start_z;
    double end_x;
    double end_y;
    double end_z;
    int32_t cyl_id;
    int32_t parent_id;
    int32_t axis_id;
    int32_t branch_order;
    int32_t quality;
    double radius;
    double dist_to_root;
    double subtree_length;
} ArborBridgeCylinder;

typedef struct ArborBridgeQSM {
    int32_t id;
    char *name;
    char *crs;
    char **messages;
    size_t message_count;
    ArborBridgeCylinder *cylinders;
    size_t cylinder_count;
} ArborBridgeQSM;

typedef struct ArborBridgeQSF {
    ArborBridgeQSM *models;
    size_t count;
} ArborBridgeQSF;

typedef struct ArborBridgeDBH {
    double dbh;
    double x;
    double y;
    double z;
    double nx;
    double ny;
    double nz;
} ArborBridgeDBH;

int arbor_bridge_segment_ground(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error);
int arbor_bridge_segment_semantic(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error);
int arbor_bridge_segment_instance(const ArborBridgeCloud *cloud, const ArborBridgeCloud *seeds, const ArborBridgeParams *params, ArborBridgeAttributes *out, char **error);
int arbor_bridge_find_seeds(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeSeedCloud *out, char **error);
int arbor_bridge_homogeneization(const ArborBridgeCloud *cloud, double res, uint8_t *keep, char **error);
int arbor_bridge_wood_likelihood(const ArborBridgeCloud *cloud, int32_t k, float *pwood, char **error);
int arbor_bridge_colorize(const ArborBridgeCloud *cloud, int darken_foliage, ArborBridgeAttributes *out, char **error);

int arbor_bridge_qsm(const ArborBridgeCloud *cloud, const ArborBridgeParams *params, ArborBridgeQSM *out, char **error);
int arbor_bridge_qsf(const ArborBridgeCloud *cloud, double min_height, const ArborBridgeParams *params, ArborBridgeQSF *out, char **error);
int arbor_bridge_qsm_dbh(const ArborBridgeQSM *model, double breast_height, ArborBridgeDBH *out, char **error);
int arbor_bridge_qsm_stem(const ArborBridgeQSM *model, ArborBridgeQSM *out, char **error);
int arbor_bridge_qsm_merchantable(const ArborBridgeQSM *model, double min_radius, double min_axis_length, ArborBridgeQSM *out, char **error);
int arbor_bridge_qsm_read(const char *path, ArborBridgeQSM *out, char **error);
int arbor_bridge_qsm_write(const ArborBridgeQSM *model, const char *path, int binary, char **error);
int arbor_bridge_qsf_read(const char *path, ArborBridgeQSF *out, char **error);
int arbor_bridge_qsf_write(const ArborBridgeQSM *models, size_t count, const char *path, const char *format, int binary, char **error);

void arbor_bridge_attributes_free(ArborBridgeAttributes *value);
void arbor_bridge_seeds_free(ArborBridgeSeedCloud *value);
void arbor_bridge_qsm_free(ArborBridgeQSM *value);
void arbor_bridge_qsf_free(ArborBridgeQSF *value);
void arbor_bridge_string_free(char *value);

#ifdef __cplusplus
}
#endif

#endif
