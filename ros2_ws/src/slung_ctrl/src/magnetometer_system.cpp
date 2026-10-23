// Fixed-site magnetic environment: world ENU [tesla] -> sensor FLU [tesla].
// Uses Gazebo's sensor noise and update rate, without replacing the world field
// with Harmonic's independent geomagnetic lookup table.
#include <gz/plugin/Register.hh>
#include <gz/sensors/MagnetometerSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/MagneticField.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Noise.hh>
#include <sdf/Sensor.hh>
#include <memory>
#include <unordered_map>

namespace slung_ctrl {
class MagnetometerSystem : public gz::sim::System,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate {
public:
    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &ecm) override {
        for (const auto &entry : sensors_) {
            if (!ecm.Component<gz::sim::components::SensorTopic>(entry.first))
                ecm.CreateComponent(entry.first,
                    gz::sim::components::SensorTopic(entry.second->Topic()));
        }
    }

    void PostUpdate(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm) override {
        const auto world = ecm.EntityByComponents(gz::sim::components::World());
        const auto *field = ecm.Component<gz::sim::components::MagneticField>(world);
        if (!field || info.paused) return;

        ecm.Each<gz::sim::components::Magnetometer, gz::sim::components::ParentEntity>(
            [&](const gz::sim::Entity entity,
                const gz::sim::components::Magnetometer *component,
                const gz::sim::components::ParentEntity *parent) {
                auto it = sensors_.find(entity);
                if (it == sensors_.end()) {
                    sdf::Sensor data = component->Data();
                    data.SetName(gz::sim::removeParentScope(
                        gz::sim::scopedName(entity, ecm, "::", false), "::"));
                    if (data.Topic().empty())
                        data.SetTopic(gz::sim::scopedName(entity, ecm) + "/magnetometer");
                    // PX4 Harmonic stock-model noise numbers act in gauss in
                    // the old plugin. Convert their units, preserving the
                    // physical noise (e.g. 0.0001 G = 1e-8 T), not reducing it.
                    auto *mag = data.MagnetometerSensor();
                    mag->SetXNoise(tesla_noise(mag->XNoise()));
                    mag->SetYNoise(tesla_noise(mag->YNoise()));
                    mag->SetZNoise(tesla_noise(mag->ZNoise()));
                    auto sensor = factory_.CreateSensor<gz::sensors::MagnetometerSensor>(data);
                    if (!sensor) return true;
                    if (const auto *name = ecm.Component<gz::sim::components::Name>(parent->Data()))
                        sensor->SetParent(name->Data());
                    it = sensors_.emplace(entity, std::move(sensor)).first;
                }
                auto &sensor = *it->second;
                sensor.SetWorldMagneticField(field->Data());
                sensor.SetWorldPose(gz::sim::worldPose(entity, ecm));
                if (sensor.HasConnections() && sensor.NextDataUpdateTime() <= info.simTime)
                    sensor.Update(info.simTime, false);
                return true;
            });
        for (auto it = sensors_.begin(); it != sensors_.end();) {
            if (!ecm.HasEntity(it->first)) it = sensors_.erase(it);
            else ++it;
        }
    }

private:
    static sdf::Noise tesla_noise(sdf::Noise noise) {
        constexpr double gauss_to_tesla = 1e-4;
        noise.SetMean(noise.Mean() * gauss_to_tesla);
        noise.SetStdDev(noise.StdDev() * gauss_to_tesla);
        noise.SetBiasMean(noise.BiasMean() * gauss_to_tesla);
        noise.SetBiasStdDev(noise.BiasStdDev() * gauss_to_tesla);
        noise.SetDynamicBiasStdDev(noise.DynamicBiasStdDev() * gauss_to_tesla);
        noise.SetPrecision(noise.Precision() * gauss_to_tesla);
        return noise;
    }
    gz::sensors::SensorFactory factory_;
    std::unordered_map<gz::sim::Entity,
        std::unique_ptr<gz::sensors::MagnetometerSensor>> sensors_;
};
} // namespace slung_ctrl

GZ_ADD_PLUGIN(slung_ctrl::MagnetometerSystem, gz::sim::System,
              slung_ctrl::MagnetometerSystem::ISystemPreUpdate,
              slung_ctrl::MagnetometerSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(slung_ctrl::MagnetometerSystem, "slung_ctrl::MagnetometerSystem")
