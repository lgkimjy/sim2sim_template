/**
 * @file: traj_viz_util.hpp -> temporal name
 * @author: Jun Young Kim ( lgkimjy@kist.re.kr )
 * @department: Center for Humanoid Research Group
 *              @ Korea Institute of Science Technology (KIST)
 * @date: 2024. 11. 20.
 * 
 * @description: Custom Trajectory Visualizer for MuJoCo
 */


#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <utility>
#include <algorithm>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>
// #include <mujoco/mjui.h>
// #include "platform_ui_adapter.h"

namespace mujoco
{
class TrajVizUtil
{
public:
    TrajVizUtil() {
        std::cout << "[ TrajVizUtil ] Custom Visualizer Constructed" << std::endl;
    };
    ~TrajVizUtil() {
        std::cout << "[ TrajVizUtil ] Destructor" << std::endl;
    };

    mjtNum p_zmp[3];
    mjtNum p_dcm[3];
    mjtNum p_com_projected[3];

    mjtNum R_B[9];
    mjtNum R_B_d[9];
    std::deque<mjtNum *> R_B_traj; // history of R_B trajectory
    std::deque<mjtNum *> R_B_traj_d; // history of R_B_d trajectory

    mjtNum R_WBO[9];
    mjtNum p_B[3];
    mjtNum p_com[3];
    mjtNum p_com_d[3];
    std::deque<mjtNum *> com_traj;    // history of com trajectory
    std::deque<mjtNum *> com_traj_d;  // history of com trajectory desired
    std::deque<mjtNum *> com_horizon; // future com trajectory
    std::deque<mjtNum *> R_horizon; // future com trajectory

    mjtNum p_foot[3];
    mjtNum p_foot_d[3];
    std::deque<mjtNum *> foot_traj;
    std::deque<mjtNum *> foot_traj_d;

    mjtNum footsteps[3];
    std::vector<mjtNum*> footsteps_planned;
    std::deque<mjtNum *> zmp_ref;

    mjtNum vertices[3];
    std::vector<mjtNum *> polygon;

    std::string support_state_str;
    std::string phase_state_str;

    mjtNum p_C0_left[3];
    mjtNum p_C1_left[3];
    mjtNum p_C2_left[3];
    mjtNum p_C3_left[3];
    mjtNum p_C4_left[3];
    mjtNum Rot_left[9];

    mjtNum p_C0_right[3];
    mjtNum p_C1_right[3];
    mjtNum p_C2_right[3];
    mjtNum p_C3_right[3];
    mjtNum p_C4_right[3];
    mjtNum Rot_right[9];

    mjtNum p_LH[3];
    mjtNum Rot_LH[9];
    std::deque<mjtNum *> p_LH_traj;
    std::deque<mjtNum *> Rot_LH_traj;
    mjtNum p_LH_d[3];
    mjtNum Rot_LH_d[9];
    std::deque<mjtNum *> p_LH_traj_d;
    std::deque<mjtNum *> Rot_LH_traj_d;

    mjtNum p_RH[3];
    mjtNum Rot_RH[9];
    std::deque<mjtNum *> p_RH_traj;
    std::deque<mjtNum *> Rot_RH_traj;
    mjtNum p_RH_d[3];
    mjtNum Rot_RH_d[9];
    std::deque<mjtNum *> p_RH_traj_d;
    std::deque<mjtNum *> Rot_RH_traj_d;


    mjtNum p_CoP_1[3];
    mjtNum p_CoP_2[3];
    mjtNum p_CoP_3[3];
    mjtNum p_CoP_4[3];
    mjtNum p_DCM[3];
    mjtNum p_DCM_eos1[3];
    mjtNum p_DCM_eos2[3];
    mjtNum p_DCM_eos3[3];
    mjtNum p_DCM_eos4[3];

public:
    void copyPosData(mjtNum *simArray, const Eigen::Vector3d &pos) {
        for (int i = 0; i < 3; ++i) {
            simArray[i] = pos(i);
        }
    }

    /**
     * @issue: Eigen uses column-major storage by default, while MuJoCo expects row-major order.
     * @fixed: Explicitly copy the matrix elements in row-major order to match MuJoCo's format.
     **/
    void copyRotData(mjtNum *simArray, const Eigen::Matrix3d &mat) {
        // Copy in row-major order
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                simArray[row * 3 + col] = mat(row, col);
            }
        }
    }

    void push_back_and_manage(std::deque<mjtNum *> &traj, const mjtNum *source, int history, int array_size)
    {
        mjtNum *new_array = new mjtNum[array_size];
        std::copy(source, source + array_size, new_array);

        if (traj.size() < history) {
            traj.push_back(new_array);
        }
        else if (traj.size() == history) {
            traj.push_back(new_array);
            delete[] traj.front(); // Free memory of the oldest element
            traj.pop_front();
        }
    }

    void update(mjvScene &scn)
    {
        visualizeGeom(p_zmp, NULL, scn, new mjtNum[2]{0.02, 0.02}, COLOR_YELLOW, mjGEOM_CYLINDER); // Visual for Current ZMP
        visaulizeText(p_zmp, scn, "ZMP");

        // visualizeGeom(p_com_projected, NULL, scn, new mjtNum[2]{0.02, 0.02}, COLOR_RED, mjGEOM_CYLINDER); // Visual for Current CoM Projected
        visualizeGeom(p_B, R_B, scn, new mjtNum[1]{0.00}, new float[4]{0.3, 0.0, 0.3, 1.0}, mjGEOM_SPHERE);

        visualizeGeom(p_com, R_WBO, scn, new mjtNum[1]{0.035}, new float[4]{1.0, 0.0, 0.0, 0.5}, mjGEOM_SPHERE);
        visualizeGeom(p_com_d, NULL, scn, new mjtNum[1]{0.03}, new float[4]{0.3, 0.0, 0.3, 1.0}, mjGEOM_SPHERE);          
        visualizeTrajDeque(com_traj, scn, 8, new float[4]{1.0, 0.0, 1.0, 0.3});
        visualizeTrajDeque(com_traj_d, scn, 4, new float[4]{0.3, 0.5, 1.0, 0.5});

        // visualizeGeom(p_C0_left, Rot_left, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C1_left, Rot_left, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C2_left, Rot_left, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C3_left, Rot_left, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C4_left, Rot_left, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);

        // visualizeGeom(p_C0_right, Rot_right, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C1_right, Rot_right, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C2_right, Rot_right, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C3_right, Rot_right, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        // visualizeGeom(p_C4_right, Rot_right, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);

        visualizeGeom(p_RH, Rot_RH, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        visualizeGeom(p_LH, Rot_LH, scn, new mjtNum[1]{0.005}, new float[4]{0.0, 1.0, 0.0, 1.0}, mjGEOM_SPHERE);
        if (p_RH_d[0] != 0.0) {
            visualizeGeom(p_RH_d, Rot_RH_d, scn, new mjtNum[1]{0.018}, new float[4]{0.3, 0.5, 1.0, 0.8}, mjGEOM_SPHERE);
        }
        if (p_LH_d[0] != 0.0) {
            visualizeGeom(p_LH_d, Rot_LH_d, scn, new mjtNum[1]{0.014}, new float[4]{0.3, 0.5, 1.0, 0.5}, mjGEOM_SPHERE);
        }
        // visualizeTrajDeque2(p_LH_traj, Rot_LH_traj, scn, 8, new float[4]{1.0, 0.0, 1.0, 0.3});
        // visualizeTrajDeque2(p_RH_traj, Rot_RH_traj, scn, 8, new float[4]{1.0, 0.0, 1.0, 0.3});

        visualizeTrajDeque2(p_LH_traj_d, Rot_LH_traj_d, scn, 8, new float[4]{0.3, 0.5, 1.0, 0.3});
        visualizeTrajDeque2(p_RH_traj_d, Rot_RH_traj_d, scn, 8, new float[4]{0.3, 0.5, 1.0, 0.3});

        // visualizeGeom(p_dcm, NULL, scn, new mjtNum[2]{0.02, 0.02}, COLOR_CYAN, mjGEOM_CYLINDER); // Visual for Current DCM
        // visaulizeText(p_dcm, scn, "DCM");

        visualizeTrajDeque(foot_traj_d, scn, 4, new float[4]{0.3, 0.5, 1.0, 0.5});
        visualizeTrajDeque(foot_traj, scn, 4, new float[4]{0.3, 1.0, 1.0, 0.5});

        // visualizeGeom(p_DCM, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{0.5, 0.5, 1.0, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_CoP_1, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.0, 1.0, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_CoP_2, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.0, 1.0, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_CoP_3, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.0, 1.0, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_CoP_4, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.0, 1.0, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_DCM_eos1, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.5, 0.3, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_DCM_eos2, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.5, 0.3, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_DCM_eos3, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.5, 0.3, 0.7}, mjGEOM_CYLINDER);
        // visualizeGeom(p_DCM_eos4, NULL, scn, new mjtNum[2]{0.015, 0.015}, new float[4]{1.0, 0.5, 0.3, 0.7}, mjGEOM_CYLINDER);

        visualizeTrajVec(polygon, scn, 25, COLOR_BLUE);
    }

private:

    /**
     * define extra colors here 
     **/
    float COLOR_RED[4] = {1.0, 0.0, 0.0, 1.0};
    float COLOR_GREEN[4] = {0.0, 1.0, 0.0, 1.0};
    float COLOR_BLUE[4] = {0.0, 0.0, 1.0, 1.0};
    float COLOR_YELLOW[4] = {1.0, 1.0, 0.0, 1.0};
    float COLOR_CYAN[4] = {0.0, 1.0, 1.0, 1.0};
    float COLOR_MAGENTA[4] = {1.0, 0.0, 1.0, 1.0};
    float COLOR_WHITE[4] = {1.0, 1.0, 1.0, 1.0};
    float COLOR_BLACK[4] = {0.0, 0.0, 0.0, 1.0};

    void visaulizeText(mjtNum *data, mjvScene &scn, std::string text)
    {
        // create an invisibale geom and add label on it
        mjvGeom *geom = scn.geoms + scn.ngeom++;
        // data[0] += 0.025;
        // data[1] += 0.025;
        // data[2] += 0.025;
        mjv_initGeom(geom, mjGEOM_LABEL, NULL, data, NULL, new float[4]{0.0, 0.0, 1.0, 1.0});
        strncpy(geom->label, text.c_str(), 100);
        geom->label[99] = '\0';
    }

    /**
     * uses for draw geometry with specific type
     **/
    void visualizeGrndPrjctdGeom(mjtNum *data, mjtNum *mat, mjvScene &scn, mjtNum size[], float color[], int type)
    {
        mjvGeom *geom = scn.geoms + scn.ngeom++;
        data[2] = 0.0;  // make z = 0
        mjv_initGeom(geom, type, size, data, NULL, color);

        if (mat != NULL)
        {
            // let's add frame for geom
            mjtNum sz[3], vec[3], axis[3];
            sz[0] = 0.005;
            sz[1] = 0.1;
            for (int j = 0; j < 3; j++)
            {
                mjvGeom *geom = scn.geoms + scn.ngeom++;
                mjv_initGeom(geom, mjGEOM_CYLINDER, NULL, NULL, NULL, NULL);

                // prepare axis
                for (int k = 0; k < 3; k++)
                {
                    axis[k] = (j == k ? sz[1] : 0);
                }
                mju_mulMatVec(vec, mat, axis, 3, 3);

                // create a cylinder
                mjtNum *from = data;
                mjtNum to[3];
                mju_add3(to, from, vec);
                mjv_connector(geom, mjGEOM_CYLINDER, sz[0], from, to);

                // set color: R, G or B depending on axis
                for (int k = 0; k < 3; k++)
                {
                    geom->rgba[k] = (j == k ? 0.9 : 0);
                }
                geom->rgba[3] = 1;
            }
        }
    }

    /**
     * uses for draw geometry with specific type
     **/
    void visualizeGeom(mjtNum *data, mjtNum *mat, mjvScene &scn, mjtNum size[], float color[], int type)
    {
        mjvGeom *geom = scn.geoms + scn.ngeom++;
        mjv_initGeom(geom, type, size, data, NULL, color);

        if (mat != NULL)
        {
            // let's add frame for geom
            mjtNum sz[3], vec[3], axis[3];
            sz[0] = 0.005;
            sz[1] = 0.1;
            for (int j = 0; j < 3; j++)
            {
                mjvGeom *geom = scn.geoms + scn.ngeom++;
                mjv_initGeom(geom, mjGEOM_CYLINDER, NULL, NULL, NULL, NULL);

                // prepare axis
                for (int k = 0; k < 3; k++)
                {
                    axis[k] = (j == k ? sz[1] : 0);
                }
                mju_mulMatVec(vec, mat, axis, 3, 3);

                // create a cylinder
                mjtNum *from = data;
                mjtNum to[3];
                mju_add3(to, from, vec);
                mjv_connector(geom, mjGEOM_CYLINDER, sz[0], from, to);

                // set color: R, G or B depending on axis
                for (int k = 0; k < 3; k++)
                {
                    geom->rgba[k] = (j == k ? 0.9 : 0);
                }
                geom->rgba[3] = 1;
            }
        }
    }

    /**
     * uses for draw foot position and orientation, with foot size
     **/
    void visualizeFoot(mjtNum *data, mjtNum* orientation, mjvScene &scn, mjtNum size[], float color[])
    {
        mjvGeom *geom = scn.geoms + scn.ngeom++;
        data[2] = 0.001;  // make z = 0.001
        // mjv_initGeom(geom, mjGEOM_BOX, size, data, orientation, color);

        // <site name="RContact1" type="sphere" group="3" pos="0.0325 0.0 -0.045" size="0.005" rgba="0 1 0 1"/>
        // <site name="RContact2" type="sphere" group="3" pos="0.075 0.04 -0.045" size="0.005" rgba="0 1 0 1"/>
        // <site name="RContact3" type="sphere" group="3" pos="-0.01 0.04 -0.045" size="0.005" rgba="0 1 0 1"/>
        // <site name="RContact4" type="sphere" group="3" pos="0.075 -0.04 -0.045" size="0.005" rgba="0 1 0 1"/>
        // <site name="RContact5" type="sphere" group="3" pos="-0.01 -0.04 -0.045" size="0.005" rgba="0 1 0 1"/>

        // prestoe foot print -> no orientation computation added, it's todo
        std::vector<mjtNum *> traj;
        traj.push_back(data);   // first foot print heel
        mjtNum data2[3] = {0.14, 0.04, 0.001}; // pos offset from data
        data2[0] += data[0];
        data2[1] += data[1];
        traj.push_back(data2);
        mjtNum data3[3] = {0.09, 0.0, 0.001}; // pos offset from data2
        data3[0] += data2[0];
        data3[1] += data2[1];
        traj.push_back(data3);
        mjtNum data4[3] = {0.0, -0.08, 0.001}; // pos offset from data3
        data4[0] += data3[0];
        data4[1] += data3[1];
        traj.push_back(data4);
        mjtNum data5[3] = {-0.09, 0.0, 0.001}; // pos offset from data4
        data5[0] += data4[0];
        data5[1] += data4[1];
        traj.push_back(data5);
        traj.push_back(data);
        visualizeTrajVec(traj, scn, 3, new float[4]{0.0, 1.0, 0.0, 0.7});
    }

    /**
     * uses for draw pre-defined foot print (position and orientation), with foot size
     **/
    void visualizeFootPrint(const std::vector<mjtNum *> &data, const std::vector<mjtNum *> &mat, mjvScene &scn, mjtNum size[], float color[])
    {
        for (int i = 0; i < data.size(); i++) {
            mjtNum matt[9];
            copyRotData(matt, Eigen::Matrix3d::Identity());
            visualizeFoot(data[i], matt, scn, size, color);
        }
    }

    /*
    uses for drawing multiple trajectories with different colors, from mppi control output
    */
    void visualizeMPPITraj(const std::deque<std::pair<std::vector<mjtNum *>, double>> &traj,
                            mjvScene &scn, int size,
                            double min_reward, double max_reward)
    {
        float color[4];

        for (int j = 0; j < traj.size(); j++)
        {
            // Normalize reward to [0,1]
            double normalized = (traj[j].second - min_reward) / (max_reward - min_reward);
            for (int i = 1; i < traj[j].first.size(); i++)
            {
                color[0] = normalized;       // Red
                color[1] = 0.0;              // Green
                color[2] = 1.0 - normalized; // Blue
                color[3] = normalized;       // Alpha
                mjvGeom *test_geom = scn.geoms + scn.ngeom++;
                mjv_initGeom(test_geom, mjGEOM_SPHERE, NULL, NULL, NULL, color);
                mjv_connector(test_geom, mjGEOM_LINE, size, traj[j].first[i - 1], traj[j].first[i]);
            }
        }
    }

    /**
     * uses for draw history of desired and Reference Trajectory with using vector
     **/
    void visualizeTrajVec(const std::vector<mjtNum *> &traj, mjvScene &scn, int size, float color[])
    {
        for (int i = 1; i < traj.size(); i++)
        {
            mjvGeom *test_geom = scn.geoms + scn.ngeom++;
            mjv_initGeom(test_geom, mjGEOM_SPHERE, NULL, NULL, NULL, color);
            mjv_connector(test_geom, mjGEOM_LINE, size, traj[i - 1], traj[i]);
        }
    }

    /**
     * uses for draw history of desired and Reference Trajectory with using deque
     **/
    void visualizeTrajDeque(const std::deque<mjtNum *> &traj, mjvScene &scn, int size, float color[])
    {
        for (int i = 1; i < traj.size(); i++)
        {
            mjvGeom *test_geom = scn.geoms + scn.ngeom++;
            mjv_initGeom(test_geom, mjGEOM_SPHERE, NULL, NULL, NULL, color);
            mjv_connector(test_geom, mjGEOM_LINE, size, traj[i - 1], traj[i]);
        }
    }

    /**
     * uses for draw history of desired and Reference Trajectory and rotation frames with using deque
     **/
    void visualizeTrajDeque2(const std::deque<mjtNum *> &traj, std::deque<mjtNum *> &R_traj, mjvScene &scn, int size, float color[])
    {
        /// @bug: if error occurs, check the size of traj and R_traj, they must be the same
        ///         traj can be smaller than R_traj, but R_traj cannot be smaller than traj
        /// @note: the interval of drawing R_traj is 50
        for (int i = 1; i < traj.size(); i++)
        {
            // Draw trajectory
            mjvGeom *test_geom = scn.geoms + scn.ngeom++;
            mjv_initGeom(test_geom, mjGEOM_SPHERE, NULL, NULL, NULL, color);
            mjv_connector(test_geom, mjGEOM_LINE, size, traj[i - 1], traj[i]);

            // Draw R_traj (rotation frames) at aligned indices
            int interval = 500;
            if (!R_traj.empty() && i % interval == 0)
            {
                int aligned_index = i / interval; // Compute
                if (aligned_index < R_traj.size() && R_traj[aligned_index] != NULL)
                {
                    // Add frame for geom
                    mjtNum sz[3], vec[3], axis[3];
                    sz[0] = 0.005; // Cylinder radius
                    sz[1] = 0.1;   // Cylinder length
                    for (int j = 0; j < 3; j++)
                    {
                        mjvGeom *geom = scn.geoms + scn.ngeom++;
                        mjv_initGeom(geom, mjGEOM_CYLINDER, NULL, NULL, NULL, NULL);

                        // Prepare axis direction (X, Y, Z)
                        for (int k = 0; k < 3; k++)
                        {
                            axis[k] = (j == k ? sz[1] : 0); // Axis vector
                        }
                        mju_mulMatVec(vec, R_traj[aligned_index * interval], axis, 3, 3); // Rotate axis
                        mju_add3(vec, traj[aligned_index * interval], vec);               // Translate to position

                        // Create a cylinder from traj[i] to the translated point
                        mjv_connector(geom, mjGEOM_CYLINDER, sz[0], traj[aligned_index * interval], vec);

                        // Set color: R, G, or B depending on the axis
                        for (int k = 0; k < 3; k++)
                        {
                            geom->rgba[k] = (j == k ? 0.9 : 0); // Red for X, Green for Y, Blue for Z
                        }
                        // geom->rgba[3] = 0.8; // Transparency
                        geom->rgba[3] = color[3]; // Transparency
                    }
                }
            }
        }
    }
};
}
