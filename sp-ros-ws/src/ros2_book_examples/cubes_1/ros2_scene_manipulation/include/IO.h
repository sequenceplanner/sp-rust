#ifndef IO_H
#define IO_H

#include <cereal/cereal.hpp>

#include "related_transform.h"

namespace IO {

template<typename TInputArchive, typename TStream>
std::vector<RelatedTransform> input(TStream& stream)
{
	TInputArchive input_archive(stream);
	std::vector<RelatedTransform> related_transforms;
	input_archive(CEREAL_NVP(related_transforms));
	return related_transforms;
}

template<typename TOutputArchive, typename TStream>
void output(TStream& stream, const std::vector<RelatedTransform>& related_transforms)
{
	TOutputArchive output_archive(stream);
	output_archive(CEREAL_NVP(related_transforms));
}

} // namespace IO

#endif