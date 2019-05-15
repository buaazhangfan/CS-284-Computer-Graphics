#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
    this->width = width;
    this->height = height;
    this->num_width_points = num_width_points;
    this->num_height_points = num_height_points;
    this->thickness = thickness;

    buildGrid();
    buildClothMesh();
}

Cloth::~Cloth() {
    point_masses.clear();
    springs.clear();

    if (clothMesh) {
        delete clothMesh;
    }
}

void Cloth::buildGrid() {
    // TODO (Part 1): Build a grid of masses and springs.

    double width_unit = width / num_width_points;
    double height_unit = height / num_height_points;
    Vector3D pos;

    for (int j = 0; j < num_height_points; j++)
    {
        for (int i = 0; i < num_width_points; i++)
        {
            if (orientation == HORIZONTAL)
            {
                pos.x = width_unit * i;
                pos.y = 1.0;
                pos.z = height_unit * j;
            }
            else
            {
                double z = -0.001 + static_cast <double> (rand()) /(RAND_MAX/(0.002));
                // double z = (double)rand() / (500.0*RAND_MAX) - (1.0/1000.0);
                pos.x = width_unit * i;
                pos.y = height_unit * j;
                pos.z = z;
            }

            vector<int>idx = {i, j};
            if (find(pinned.begin(), pinned.end(), idx) != pinned.end())
            {
                PointMass pm(pos, true);
                point_masses.emplace_back(pm);
            }
            else
            {
                PointMass pm(pos, false);
                point_masses.emplace_back(pm);
            }

        }
    }


    for (int j = 0; j < num_height_points; j++)
    {
        for (int i = 0; i < num_width_points; i++)
        {
            int pm_idx = j * num_width_points + i;
            int str_1_x = i - 1, str_1_y = j, str_2_x = i, str_2_y = j - 1;
            int she_1_x = i - 1, she_1_y = j - 1, she_2_x = i + 1, she_2_y = j - 1;
            int ben_1_x = i - 2, ben_1_y = j, ben_2_x = i, ben_2_y = j - 2;

            if (avail_idx(str_1_x, str_1_y, num_width_points, num_height_points))
            {
                int neig_idx = str_1_y * num_width_points + str_1_x;
                Spring sp1(&point_masses[pm_idx], &point_masses[neig_idx], STRUCTURAL);
                springs.emplace_back(sp1);
            }
            if (avail_idx(str_2_x, str_2_y, num_width_points, num_height_points))
            {
                int neig_idx = str_2_y * num_width_points + str_2_x;
                Spring sp2(&point_masses[pm_idx], &point_masses[neig_idx], STRUCTURAL);
                springs.emplace_back(sp2);
            }
            if (avail_idx(she_1_x, she_1_y, num_width_points, num_height_points))
            {
                int neig_idx = she_1_y * num_width_points + she_1_x;
                Spring sp3(&point_masses[pm_idx], &point_masses[neig_idx], SHEARING);
                springs.emplace_back(sp3);
            }
            if (avail_idx(she_2_x, she_2_y, num_width_points, num_height_points))
            {
                int neig_idx = she_2_y * num_width_points + she_2_x;
                Spring sp4(&point_masses[pm_idx], &point_masses[neig_idx], SHEARING);
                springs.emplace_back(sp4);
            }
            if (avail_idx(ben_1_x, ben_1_y, num_width_points, num_height_points))
            {
                int neig_idx = ben_1_y * num_width_points + ben_1_x;
                Spring sp5(&point_masses[pm_idx], &point_masses[neig_idx], BENDING);
                springs.emplace_back(sp5);
            }
            if (avail_idx(ben_2_x, ben_2_y, num_width_points, num_height_points))
            {
                int neig_idx = ben_2_y * num_width_points + ben_2_x;
                Spring sp6(&point_masses[pm_idx], &point_masses[neig_idx], BENDING);
                springs.emplace_back(sp6);
            }
        }
    }


}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    double mass = width * height * cp->density / num_width_points / num_height_points;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    // TODO (Part 2): Compute total force acting on each point mass.

    Vector3D total_a(0.0, 0.0, 0.0);
    for (auto &a : external_accelerations)
    {
        total_a += a;
    }

    for (auto &pm : point_masses)
    {
        // pm.forces = Vector3D(0, 0, 0);
        pm.forces = mass * total_a;
    }

    for (auto &sp : springs)
    {
        if (cp -> enable_structural_constraints && sp.spring_type == STRUCTURAL)
        {
            Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
            double distance = dir.norm();
            dir.normalize();
            sp.pm_a -> forces += cp -> ks * (distance - sp.rest_length) * (-1.0 * dir);
            sp.pm_b -> forces += cp -> ks * (distance - sp.rest_length) * (1.0 * dir);

        }
        if (cp -> enable_shearing_constraints && sp.spring_type == SHEARING)
        {
            Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
            double distance = dir.norm();
            dir.normalize();
            sp.pm_a -> forces += cp -> ks * (distance - sp.rest_length) * (-1.0 * dir);
            sp.pm_b -> forces += cp -> ks * (distance - sp.rest_length) * (1.0 * dir);

        }
        if (cp -> enable_bending_constraints && sp.spring_type == BENDING)
        {
            Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
            double distance = dir.norm();
            dir.normalize();
            sp.pm_a -> forces += cp -> ks * (distance - sp.rest_length) * (-1.0 * dir);
            sp.pm_b -> forces += cp -> ks * (distance - sp.rest_length) * (1.0 * dir);
//            sp.pm_a -> forces += 0.2 * cp -> ks * (distance - sp.rest_length) * (-1.0 * dir);
//            sp.pm_b -> forces += 0.2 * cp -> ks * (distance - sp.rest_length) * (1.0 * dir);

        }
    }

    // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (auto &pm : point_masses)
    {
        if (!pm.pinned)
        {
            Vector3D overall_a = pm.forces / mass;
            Vector3D new_pos = pm.position + (1 - cp -> damping / 100.0)
                                * (pm.position - pm.last_position) + overall_a * delta_t * delta_t;
            pm.last_position = pm.position;
            pm.position = new_pos;
        }
    }

    // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (auto &pm : point_masses)
    {
        self_collide(pm, simulation_steps);
    }

    // TODO (Part 3): Handle collisions with other primitives.
    for (auto &pm : point_masses)
    {
        for (auto *co : *collision_objects)
        {
            co -> collide(pm);
        }
    }

    // TODO (Part 2): Constrain the changes to be such that the spring does not change
    // in length more than 10% per timestep [Provot 1995].
    for (auto &sp : springs)
    {
        double dis = (sp.pm_a -> position - sp.pm_b -> position).norm();
        double diff = dis - sp.rest_length * 1.1;

        if (diff > 0) {
            if (sp.pm_b->pinned && !sp.pm_a->pinned)
            {
                Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
                dir.normalize();
                Vector3D move_dis = (-1.0 * dir) * diff;
                sp.pm_a -> position += move_dis;
            }

            if (!sp.pm_b->pinned && sp.pm_a->pinned)
            {
                Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
                dir.normalize();
                Vector3D move_dis = dir * diff;
                sp.pm_b -> position += move_dis;
            }

            if (!sp.pm_b->pinned && !sp.pm_a->pinned)
            {
                double half = diff / 2.0;
                Vector3D dir = sp.pm_a -> position - sp.pm_b -> position;
                dir.normalize();
                Vector3D move_dis_a = (-1.0 * dir) * half;
                Vector3D move_dis_b = dir * half;
                sp.pm_a -> position += move_dis_a;
                sp.pm_b -> position += move_dis_b;
            }
        }

    }
}

void Cloth::build_spatial_map() {
    for (const auto &entry : map) {
        delete(entry.second);
    }
    map.clear();

    // TODO (Part 4): Build a spatial map out of all of the point masses.
    for (auto &pm : point_masses)
    {
        float hash_value = hash_position(pm.position);
        if (map.find(hash_value) == map.end())
        {
            map[hash_value] = new vector<PointMass *>();
        }

        map[hash_value] -> emplace_back(&pm);
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
    // TODO (Part 4): Handle self-collision for a given point mass.
    float hash_value = hash_position(pm.position);
    // cout << "Current hash_value has " << map[hash_value] -> size() << endl;
    Vector3D correction(0.0, 0.0, 0.0);
    int count = 0.0;
    for (auto *ppm : *map[hash_value])
    {
        if (!(ppm -> start_position == pm.start_position))
        {
            double dis = (ppm -> position - pm.position).norm();
            if (dis < 2 * thickness)
            {
                double diff = 2 * thickness - dis;
                Vector3D dir = pm.position - ppm -> position;
                dir.normalize();
                dir = dir * diff;
                count += 1;
                correction += dir;
            }
        }
    }

    if (count != 0.0)
    {
        correction = correction / count / simulation_steps;
        pm.position += correction;
    }
//    Vector3D correction;
//    for (PointMass *pointMass : *map[hash_position(pm.position)]) {
//        double dist = (pm.position - pointMass->position).norm();
//        if (dist != 0 && dist < 2 * thickness) {
//            correction += ((pm.position - pointMass->position) * ((2 * thickness / dist) - 1));
//        }
//    }
//    pm.position += (correction / simulation_steps);
}

float Cloth::hash_position(Vector3D pos) {
    // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3.0 * width / num_width_points;
    double h = 3.0 * height / num_height_points;
    double t = max(w, h);

    double tru_x = (pos.x - fmod(pos.x, w)) / w;
    double tru_y = (pos.y - fmod(pos.y, h)) / h;
    double tru_z = (pos.z - fmod(pos.z, t)) / t;
    int prime = 379;
    return tru_x * prime + tru_y * prime * prime + tru_z * prime * prime * prime;
//    return tru_x + tru_y * num_width_points + tru_z * num_width_points * num_height_points;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
    PointMass *pm = &point_masses[0];
    for (int i = 0; i < point_masses.size(); i++) {
        pm->position = pm->start_position;
        pm->last_position = pm->start_position;
        pm++;
    }
}

void Cloth::buildClothMesh() {
    if (point_masses.size() == 0) return;

    ClothMesh *clothMesh = new ClothMesh();
    vector<Triangle *> triangles;

    // Create vector of triangles
    for (int y = 0; y < num_height_points - 1; y++) {
        for (int x = 0; x < num_width_points - 1; x++) {
            PointMass *pm = &point_masses[y * num_width_points + x];
            // Get neighboring point masses:
            /*                      *
             * pm_A -------- pm_B   *
             *             /        *
             *  |         /   |     *
             *  |        /    |     *
             *  |       /     |     *
             *  |      /      |     *
             *  |     /       |     *
             *  |    /        |     *
             *      /               *
             * pm_C -------- pm_D   *
             *                      *
             */

            float u_min = x;
            u_min /= num_width_points - 1;
            float u_max = x + 1;
            u_max /= num_width_points - 1;
            float v_min = y;
            v_min /= num_height_points - 1;
            float v_max = y + 1;
            v_max /= num_height_points - 1;

            PointMass *pm_A = pm                       ;
            PointMass *pm_B = pm                    + 1;
            PointMass *pm_C = pm + num_width_points    ;
            PointMass *pm_D = pm + num_width_points + 1;

            Vector3D uv_A = Vector3D(u_min, v_min, 0);
            Vector3D uv_B = Vector3D(u_max, v_min, 0);
            Vector3D uv_C = Vector3D(u_min, v_max, 0);
            Vector3D uv_D = Vector3D(u_max, v_max, 0);


            // Both triangles defined by vertices in counter-clockwise orientation
            triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
                                             uv_A, uv_C, uv_B));
            triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
                                             uv_B, uv_C, uv_D));
        }
    }

    // For each triangle in row-order, create 3 edges and 3 internal halfedges
    for (int i = 0; i < triangles.size(); i++) {
        Triangle *t = triangles[i];

        // Allocate new halfedges on heap
        Halfedge *h1 = new Halfedge();
        Halfedge *h2 = new Halfedge();
        Halfedge *h3 = new Halfedge();

        // Allocate new edges on heap
        Edge *e1 = new Edge();
        Edge *e2 = new Edge();
        Edge *e3 = new Edge();

        // Assign a halfedge pointer to the triangle
        t->halfedge = h1;

        // Assign halfedge pointers to point masses
        t->pm1->halfedge = h1;
        t->pm2->halfedge = h2;
        t->pm3->halfedge = h3;

        // Update all halfedge pointers
        h1->edge = e1;
        h1->next = h2;
        h1->pm = t->pm1;
        h1->triangle = t;

        h2->edge = e2;
        h2->next = h3;
        h2->pm = t->pm2;
        h2->triangle = t;

        h3->edge = e3;
        h3->next = h1;
        h3->pm = t->pm3;
        h3->triangle = t;
    }

    // Go back through the cloth mesh and link triangles together using halfedge
    // twin pointers

    // Convenient variables for math
    int num_height_tris = (num_height_points - 1) * 2;
    int num_width_tris = (num_width_points - 1) * 2;

    bool topLeft = true;
    for (int i = 0; i < triangles.size(); i++) {
        Triangle *t = triangles[i];

        if (topLeft) {
            // Get left triangle, if it exists
            if (i % num_width_tris != 0) { // Not a left-most triangle
                Triangle *temp = triangles[i - 1];
                t->pm1->halfedge->twin = temp->pm3->halfedge;
            } else {
                t->pm1->halfedge->twin = nullptr;
            }

            // Get triangle above, if it exists
            if (i >= num_width_tris) { // Not a top-most triangle
                Triangle *temp = triangles[i - num_width_tris + 1];
                t->pm3->halfedge->twin = temp->pm2->halfedge;
            } else {
                t->pm3->halfedge->twin = nullptr;
            }

            // Get triangle to bottom right; guaranteed to exist
            Triangle *temp = triangles[i + 1];
            t->pm2->halfedge->twin = temp->pm1->halfedge;
        } else {
            // Get right triangle, if it exists
            if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
                Triangle *temp = triangles[i + 1];
                t->pm3->halfedge->twin = temp->pm1->halfedge;
            } else {
                t->pm3->halfedge->twin = nullptr;
            }

            // Get triangle below, if it exists
            if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
                Triangle *temp = triangles[i + num_width_tris - 1];
                t->pm2->halfedge->twin = temp->pm3->halfedge;
            } else {
                t->pm2->halfedge->twin = nullptr;
            }

            // Get triangle to top left; guaranteed to exist
            Triangle *temp = triangles[i - 1];
            t->pm1->halfedge->twin = temp->pm2->halfedge;
        }

        topLeft = !topLeft;
    }

    clothMesh->triangles = triangles;
    this->clothMesh = clothMesh;
}
