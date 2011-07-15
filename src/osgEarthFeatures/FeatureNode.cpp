/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthFeatures/FeatureNode>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureNode] "

FeatureNode::FeatureNode(FeatureSource * featureSource, FeatureID fid)
	: _featureSource(featureSource), _fid(fid)
{
}

FeatureMultiNode::FeatureMultiNode(FeatureSource * featureSource)
	: FeatureNode(featureSource)
{
}

void FeatureMultiNode::addDrawable(osg::Drawable * drawable, FeatureID fid)
{
	//OE_DEBUG << LC << "addDrawable " << drawable << " fid=" << fid << std::endl;
	_drawables.insert(DrawableFeatureIDMap::value_type(drawable, fid));
}

void FeatureMultiNode::removeDrawable(osg::Drawable * drawable)
{
	DrawableFeatureIDMap::iterator it = _drawables.find(drawable);
	if(it != _drawables.end())
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable << std::endl;
		_drawables.erase(it);
	}
	else
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable  << " not found" << std::endl;
	}
}

void FeatureMultiNode::clearDrawables()
{
	_drawables.clear();
}

unsigned FeatureMultiNode::getNumDrawables() const
{
	return _drawables.size();
}

void FeatureMultiNode::addPrimitiveSet(osg::PrimitiveSet * primitiveSet, FeatureID fid)
{
	//OE_DEBUG << LC << "addDrawable " << drawable << " fid=" << fid << std::endl;
	_primitiveSets.insert(PrimitiveSetFeatureIDMap::value_type(primitiveSet, fid));
}

void FeatureMultiNode::removePrimitiveSet(osg::PrimitiveSet * primitiveSet)
{
	PrimitiveSetFeatureIDMap::iterator it = _primitiveSets.find(primitiveSet);
	if(it != _primitiveSets.end())
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable << std::endl;
		_primitiveSets.erase(it);
	}
	else
	{
		//OE_DEBUG << LC << "FeatureMultiNode removeDrawable " << drawable  << " not found" << std::endl;
	}
}

void FeatureMultiNode::clearPrimitiveSets()
{
	_primitiveSets.clear();
}

unsigned FeatureMultiNode::getNumPrimitiveSets() const
{
	return _primitiveSets.size();
}

FeatureID FeatureMultiNode::getFID(osg::PrimitiveSet * primitiveSet) const
{
	PrimitiveSetFeatureIDMap::const_iterator it = _primitiveSets.find(primitiveSet);
	FeatureID ret = (it != _primitiveSets.end())?it->second:-1;
	//OE_DEBUG << LC << "FeatureMultiNode getFID " << drawable << " fid=" << ret << std::endl;
	return ret;
}

FeatureID FeatureMultiNode::getFID(osg::Drawable * drawable, int primitiveIndex) const
{
	FeatureID ret;
	DrawableFeatureIDMap::const_iterator it = _drawables.find(drawable);
	if(it != _drawables.end())
		ret = it->second;
	else if(primitiveIndex >= 0)
	{
		osg::Geometry * geometry = drawable->asGeometry();
		osg::PrimitiveSet * primitiveSet = geometry->getPrimitiveSet(primitiveIndex);
		PrimitiveSetFeatureIDMap::const_iterator it = _primitiveSets.find(primitiveSet);
		ret = (it != _primitiveSets.end())?it->second:-1;
	}
	else
		ret = -1;
	return ret;
}
