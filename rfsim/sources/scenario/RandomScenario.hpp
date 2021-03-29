#include <chrono>
#include <glm/gtc/constants.hpp>
#include <logic/GameScenario.hpp>
#include <random>
#include <scenario/ScenarioCommon.hpp>

namespace rfsim {
    class RandomScenario : public GameScenario {
    public:
        static const int MAX_MOTOR_FORCE = 100;

        ~RandomScenario() override = default;

        std::string GetName() const override { return "Random Scenario"; }

        std::string GetDescription() const override {
            return "RandomScenario robot placement";
        }

        std::shared_ptr<Game> CreateGame() const override {
            std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
            auto dist = std::uniform_real_distribution<float>(0.0, 1.0);

            auto game = std::make_shared<Game>();

            // Some constants
            const auto pi = glm::pi<float>();
            const auto fieldSize = ScenarioCommon::GetDefaultFieldSize();
            const auto fieldBorder = ScenarioCommon::GetDefaultFieldBorderOffset();
            const float fieldLength = fieldSize.x;
            const float fieldWidth = fieldSize.y;

            // Team size (total x2 robots)
            game->teamSize = ScenarioCommon::DEFAULT_TEAM_SIZE;

            auto &physicsProperties = game->physicsGameProperties;
            physicsProperties = ScenarioCommon::GetDefaultPhysicsProperties();

            // Field settings will be fixed (but ball placement can differ)
            auto &beginInfo = game->physicsGameInitInfo;
            beginInfo.fieldTopLeftBounds = {fieldBorder.x, fieldBorder.y};
            beginInfo.fieldBottomRightBounds = {fieldLength - fieldBorder.x,
                                                fieldWidth - fieldBorder.y};
            beginInfo.roomTopLeftBounds = {0, 0};
            beginInfo.roomBottomRightBounds = {fieldLength, fieldWidth};
            beginInfo.ballPosition = {fieldLength * 0.75f, fieldWidth * 0.75f};

            // Initial robots placement
            for (int i = 0; i < game->teamSize; i++) {
                if (i == 0) {
                    // Create main robot
                    beginInfo.robotsTeamA.push_back({0, {0.2 * fieldLength, 0.2 * fieldWidth}, 0});
                    game->robotWheelVelocitiesA.emplace_back(0, 0);
                } else {
                    beginInfo.robotsTeamA.push_back(
                            {i,
                             {dist(engine) * fieldLength, dist(engine) * fieldWidth},
                             dist(engine) * 2 * pi});
                    game->robotWheelVelocitiesA.emplace_back(0, 0);
                }

                beginInfo.robotsTeamB.push_back(
                        {(int) (i + game->teamSize),
                         {dist(engine) * fieldLength, dist(engine) * fieldWidth},
                         dist(engine) * 2 * pi});
                game->robotWheelVelocitiesB.emplace_back(0, 0);
            }

            // Graphics is exact copy of ph settings + init info
            auto &sceneSettings = game->graphicsSceneSettings;
            sceneSettings = ScenarioCommon::GetDefaultSceneSettingsFromPhysics(
                    physicsProperties, beginInfo);

            return game;
        }
    };
}// namespace rfsim