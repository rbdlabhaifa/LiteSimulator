//
// Created by tzuk on 4/19/23.
//

#include "simulator.h"
#include "auxiliary.h"
namespace LiteSimulator
{
    void LiteSimulator::simulator::drawPoints(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &offsets, float scaleFactor)
    {
        glPointSize(10);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        for (auto point : points)
        {
            glVertex3f((float)((point(0) - offsets(0)) / scaleFactor), (float)((point(1) - offsets(1)) / scaleFactor),
                       (float)((point(2) - offsets(2)) / scaleFactor));
        }
        glEnd();
    }

    void LiteSimulator::simulator::run(std::string &cloudPointsPath, std::string &modelPath, Eigen::Vector3d &startPosition, Eigen::VectorXd degreesOfFreedomOffsets, double frameWidth, double frameHeight, double fx, double fy, double cx, double cy, double scaleFactor)
    {
        {
            Eigen::Matrix3d K;
            K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
            Eigen::Vector2i viewport_size(640, 480);

            // Options
            bool show_bounds = false;
            bool show_axis = false;
            bool show_x0 = false;
            bool show_y0 = false;
            bool show_z0 = false;
            bool cull_backfaces = false;
            const float x_offset = degreesOfFreedomOffsets(0);
            const float y_offset = degreesOfFreedomOffsets(1);
            const float z_offset = degreesOfFreedomOffsets(2);
            const float yaw_offset = degreesOfFreedomOffsets(3);
            const float pitch_offset = degreesOfFreedomOffsets(4);
            const float roll_offset = degreesOfFreedomOffsets(5);
            Eigen::Vector3d offsets(x_offset, y_offset, z_offset);
            // Create Window for rendering
            pangolin::CreateWindowAndBind("Main", frameWidth, frameHeight);
            glEnable(GL_DEPTH_TEST);
            const auto mvm = pangolin::ModelViewLookAt(startPosition(0), startPosition(1), startPosition(2), 0, 0, 0, 0.0, -1.0, pangolin::AxisY);
            const auto proj = pangolin::ProjectionMatrix(viewport_size(0), viewport_size(1), K(0, 0), K(1, 1), K(0, 2), K(1, 2), 0.1, 10000);
            // Define Projection and initial ModelView matrix
            pangolin::OpenGlRenderState s_cam(proj, mvm);

            // Create Interactive View in window
            pangolin::Handler3D handler(s_cam);
            pangolin::View &d_cam = pangolin::CreateDisplay()
                                        .SetBounds(0.0, 1.0, 0.0, 1.0, -frameWidth / frameHeight)
                                        .SetHandler(&handler);

            // Load Geometry asynchronously
            const pangolin::Geometry geom_to_load = pangolin::LoadGeometry(modelPath);
            auto aabb = pangolin::GetAxisAlignedBox(geom_to_load);
            Eigen::AlignedBox3f total_aabb;
            total_aabb.extend(aabb);

            s_cam.SetModelViewMatrix(mvm);
            s_cam.SetProjectionMatrix(proj);
            const pangolin::GlGeometry geomToRender = pangolin::ToGlGeometry(geom_to_load);
            pangolin::GlSlProgram default_prog;
            auto LoadProgram = [&]()
            {
                default_prog.ClearShaders();
                default_prog.AddShader(pangolin::GlSlAnnotatedShader, shader);
                default_prog.Link();
            };
            LoadProgram();
            pangolin::RegisterKeyPressCallback('b', [&]()
                                               { show_bounds = !show_bounds; });
            pangolin::RegisterKeyPressCallback('0', [&]()
                                               { cull_backfaces = !cull_backfaces; });

            // Show axis and axis planes
            pangolin::RegisterKeyPressCallback('a', [&]()
                                               { show_axis = !show_axis; });
            pangolin::RegisterKeyPressCallback('x', [&]()
                                               { show_x0 = !show_x0; });
            pangolin::RegisterKeyPressCallback('y', [&]()
                                               { show_y0 = !show_y0; });
            pangolin::RegisterKeyPressCallback('z', [&]()
                                               { show_z0 = !show_z0; });

            Eigen::Vector3d Pick_w = handler.Selected_P_w();
            std::vector<Eigen::Vector3d> Picks_w;
            std::vector<Eigen::Vector3d> cloudPoints = auxiliary::readCsv(cloudPointsPath, ',');

            while (!pangolin::ShouldQuit())
            {
                if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6)
                {
                    Pick_w = handler.Selected_P_w();
                    Picks_w.push_back(Pick_w);
                    std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
                              << std::endl;
                }

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                if (d_cam.IsShown())
                {
                    d_cam.Activate();

                    if (cull_backfaces)
                    {
                        glEnable(GL_CULL_FACE);
                        glCullFace(GL_BACK);
                    }
                    default_prog.Bind();
                    default_prog.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
                    pangolin::GlDraw(default_prog, geomToRender, nullptr);
                    default_prog.Unbind();
                    Eigen::Matrix4d mv_mat = s_cam.GetModelViewMatrix();

                    // Compute the camera position by inverting the model-view matrix and extracting the translation component
                    Eigen::Matrix4d inv_mv_mat = mv_mat.inverse();
                    Eigen::Vector3d cam_pos = inv_mv_mat.block<3, 1>(0, 3);

                    // Compute the yaw, pitch, and roll angles from the rotation component of the model-view matrix
                    Eigen::Matrix3d rot_mat = mv_mat.block<3, 3>(0, 0);
                    double yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
                    double pitch = asin(-rot_mat(2, 0));
                    double roll = atan2(rot_mat(2, 1), rot_mat(2, 2));

                    std::cout << "Camera position: " << cam_pos << ", yaw: " << yaw << ", pitch: " << pitch << ", roll: "
                              << roll << std::endl;

                    s_cam.Apply();
                    if (show_x0)
                        pangolin::glDraw_x0(10.0, 10);
                    if (show_y0)
                        pangolin::glDraw_y0(10.0, 10);
                    if (show_z0)
                        pangolin::glDraw_z0(10.0, 10);
                    if (show_axis)
                        pangolin::glDrawAxis(10.0);
                    if (show_bounds)
                        pangolin::glDrawAlignedBox(total_aabb);

                    glDisable(GL_CULL_FACE);
                    drawPoints(cloudPoints, offsets, 3);
                }

                pangolin::FinishFrame();
            }
        }
    }
} // LiteSimulator