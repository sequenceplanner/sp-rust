#ifndef RELATED_TRANSFORM_H
#define RELATED_TRANSFORM_H

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace cereal {

template<class TArchive>
void serialize(TArchive& archive, geometry_msgs::msg::Quaternion& quaternion) {
	archive(
		cereal::make_nvp("x", quaternion.x),
		cereal::make_nvp("y", quaternion.y),
		cereal::make_nvp("z", quaternion.z),
		cereal::make_nvp("w", quaternion.w)
	);                                                                          
}

template<class TArchive>
void serialize(TArchive& archive, geometry_msgs::msg::Vector3& vector3) {
	archive(
		cereal::make_nvp("x", vector3.x),
		cereal::make_nvp("y", vector3.y),
		cereal::make_nvp("z", vector3.z)
	);                                                                          
}

template<class TArchive>
void serialize(TArchive& archive, geometry_msgs::msg::Transform& transform) {
	archive(
		cereal::make_nvp("Vector3", transform.translation),
		cereal::make_nvp("Quaternion", transform.rotation)
	);                                                                          
}

} // namespace cereal

struct RelatedTransform
{
    template<class TArchive>
	void serialize(TArchive& archive) {
        archive(
			CEREAL_NVP(frame_id),
			CEREAL_NVP(parent_id),
			CEREAL_NVP(transform)
		);                                                                          
    }
	std::string						frame_id;
	std::string						parent_id;
	geometry_msgs::msg::Transform	transform;
};

#endif