
#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
// #include <dart/utils/utils.hpp>


//==============================================================================
class simpleWidget : public dart::gui::osg::ImGuiWidget
{
	public:
		/// Constructor
		simpleWidget(
				dart::gui::osg::ImGuiViewer* viewer, dart::simulation::WorldPtr world)
			: mViewer(viewer),
			mWorld(std::move(world)),
			mGuiGravity(true),
			mGravity(true),
			mGuiHeadlights(true),
			mSolverType(-1)
	{
		// Do nothing
	}

		// Documentation inherited
		void render() override
		{
			ImGui::SetNextWindowPos(ImVec2(10, 20));
			if (!ImGui::Begin(
						"QP Impact Controller",
						nullptr,
						ImVec2(240, 320),
						0.5f,
						ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
						| ImGuiWindowFlags_HorizontalScrollbar))
			{
				// Early out if the window is collapsed, as an optimization.
				ImGui::End();
				return;
			}

			// Menu
			if (ImGui::BeginMenuBar())
			{
				if (ImGui::BeginMenu("Menu"))
				{
					if (ImGui::MenuItem("Exit"))
						mViewer->setDone(true);
					ImGui::EndMenu();
				}
				if (ImGui::BeginMenu("Help"))
				{
					if (ImGui::MenuItem("About DART"))
						mViewer->showAbout();
					ImGui::EndMenu();
				}
				ImGui::EndMenuBar();
			}

			ImGui::Text("QP Impact Demo");
			ImGui::Spacing();

			ImGui::Separator();
			ImGui::Text(
					"%.3f ms/frame (%.1f FPS)",
					1000.0f / ImGui::GetIO().Framerate,
					ImGui::GetIO().Framerate);
			ImGui::Spacing();

			if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
			{
				int e = mViewer->isSimulating() ? 0 : 1;
					if (mViewer->isAllowingSimulation())
					{
						if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
							mViewer->simulate(true);
						ImGui::SameLine();
						if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
							mViewer->simulate(false);
					}

				ImGui::Text("LCP solver:");

				static int solverType = 0;
				ImGui::RadioButton("SI", &solverType, 0);
				ImGui::RadioButton("Dantzig", &solverType, 0);
				ImGui::SameLine();
				ImGui::RadioButton("Dantzig", &solverType, 1);
				ImGui::SameLine();
				ImGui::RadioButton("PGS", &solverType, 2);
				setLcpSolver(solverType);

				ImGui::Text("Time: %.3f", mWorld->getTime());
			}

			if (ImGui::CollapsingHeader(
						"World Options", ImGuiTreeNodeFlags_DefaultOpen))
			{
				// Gravity
				ImGui::Checkbox("Gravity On/Off", &mGuiGravity);
				setGravity(mGuiGravity);

				ImGui::Spacing();

				// Headlights
				mGuiHeadlights = mViewer->checkHeadlights();
				ImGui::Checkbox("Headlights On/Off", &mGuiHeadlights);
				mViewer->switchHeadlights(mGuiHeadlights);
			}

			if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen))
			{
				osg::Vec3d eye;
				osg::Vec3d center;
				osg::Vec3d up;
				mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

				ImGui::Text("Eye   : (%.2f, %.2f, %.2f)", eye.x(), eye.y(), eye.z());
				ImGui::Text(
						"Center: (%.2f, %.2f, %.2f)", center.x(), center.y(), center.z());
				ImGui::Text("Up    : (%.2f, %.2f, %.2f)", up.x(), up.y(), up.z());
			}

			if (ImGui::CollapsingHeader("Help"))
			{
				ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
				ImGui::Text("User Guide:\n");
				ImGui::Text("%s", mViewer->getInstructions().c_str());
				ImGui::PopTextWrapPos();
			}

			ImGui::End();
		}
	protected:
		void setLcpSolver(int solverType)
		{
			if (solverType == mSolverType)
				return;

			if (solverType == 0)
			{
				auto lcpSolver = std::make_shared<dart::constraint::DantzigBoxedLcpSolver>();
				auto solver = dart::common::make_unique<dart::constraint::BoxedLcpConstraintSolver>(
						mWorld->getTimeStep(), lcpSolver);
				mWorld->setConstraintSolver(std::move(solver));
			}
			else if (solverType == 1)
			{
				auto lcpSolver = std::make_shared<dart::constraint::DantzigBoxedLcpSolver>();
				auto solver = dart::common::make_unique<dart::constraint::BoxedLcpConstraintSolver>(
						mWorld->getTimeStep(), lcpSolver);
				mWorld->setConstraintSolver(std::move(solver));
			}
			else if (solverType == 2)
			{
				auto lcpSolver = std::make_shared<dart::constraint::PgsBoxedLcpSolver>();
				auto solver = dart::common::make_unique<dart::constraint::BoxedLcpConstraintSolver>(
						mWorld->getTimeStep(), lcpSolver);
				mWorld->setConstraintSolver(std::move(solver));
			}
			else
			{
				dtwarn << "Unsupported boxed-LCP solver selected: " << solverType << "\n";
			}

			mSolverType = solverType;
		}

		void setGravity(bool gravity)
		{
			if (mGravity == gravity)
				return;

			mGravity = gravity;

			if (mGravity)
				mWorld->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
			else
				mWorld->setGravity(Eigen::Vector3d::Zero());
		}

		dart::gui::osg::ImGuiViewer* mViewer;
		dart::simulation::WorldPtr mWorld;
		bool mGuiGravity;
		bool mGravity;
		bool mGuiHeadlights;
		int mSolverType;
};

