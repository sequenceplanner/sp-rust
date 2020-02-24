#ifndef READ_TRANSFORMS_PARAM_H
#define READ_TRANSFORMS_PARAM_H

#include <sstream>
#include <iostream>
#include <algorithm>
#include <unordered_map>

#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

#include "IO.h"
#include "related_transform.h"

std::unordered_map<std::string, RelatedTransform> readTransforms(const std::string& data)
{
	//std::string contents;
	//if (!node.getParam(param_name, contents)){
		//ROS_ERROR("Could not find parameter ~/%s", param_name.c_str());
	//}

	std::stringstream ss(data);
	std::vector<RelatedTransform> related_transforms(IO::input<cereal::JSONInputArchive>(ss));

 	IO::output<cereal::JSONOutputArchive>(std::cout, related_transforms);


	//ROS_INFO_STREAM("From parameter " << param_name << " loaded transforms:");
	//for(const auto& transform : related_transforms){
		//ROS_INFO_STREAM(transform.parent_id << " -> " << transform.frame_id);
	//}

	std::unordered_map<std::string, RelatedTransform> related_transforms_map;
	std::transform(
		related_transforms.begin(),
		related_transforms.end(),
		std::inserter(related_transforms_map, related_transforms_map.end()),
		[](const RelatedTransform& related_transform) {
			return std::make_pair(related_transform.frame_id, related_transform);
		});
	return related_transforms_map;
}

#endif