/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarth/ModelLayer>
#include <osgEarth/Map>

using namespace osgEarth;

// to be depcreated
ModelLayer::ModelLayer( const std::string& name, const std::string& driver, const Config& driverConf ) :
osg::Referenced( true ),
_name( name ),
_driver( driver ),
_driverConf( driverConf ),
_enabled(true)
{
    _driverConf.attr("name") = name;
    _driverConf.attr("driver") = driver;
    _driverOptions = new DriverOptions( _driverConf );
}

ModelLayer::ModelLayer( const std::string& name, const DriverOptions* options ) :
osg::Referenced( true ),
_name( name ),
_driverOptions( options ),
_enabled(true)
{
    if (options)
        fromConfig( options->config() );
}

ModelLayer::ModelLayer( const std::string& name, ModelSource* source ) :
osg::Referenced( true ),
_name( name ),
_modelSource( source ),
_enabled(true)
{
    //NOP
}

void
ModelLayer::initialize( const std::string& referenceURI, const Map* map )
{
    _referenceURI = referenceURI;

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _driverOptions.get() );
    }

    if ( !_modelSource.valid() )
    {
        _modelSource = ModelSourceFactory::create( _name, _driver, _driverConf );
    }

    if ( _modelSource.valid() )
    {
        _modelSource->initialize( _referenceURI, map );
    }
}

osg::Node*
ModelLayer::getOrCreateNode( ProgressCallback* progress )
{
    if (!_node.valid() && _modelSource.valid())
    {
        _node = _modelSource->createNode( progress );
        //Initialize the nodemask
        _node->setNodeMask( _enabled.get() ? ~0 : 0 );
    }
    return _node.get();
}

Config
ModelLayer::toConfig() const
{
    Config conf = 
        _driverOptions.valid() ? _driverOptions->toConfig() : Config();

    conf.key() = "model";
    conf.attr("name") = _name;
    conf.updateIfSet( "enabled", _enabled );

    return conf;
}

bool
ModelLayer::getEnabled() const
{
    return _enabled.get();
}

void
ModelLayer::setEnabled(bool enabled)
{
    if (_enabled != enabled)
    {
        _enabled = enabled;
        _node->setNodeMask( _enabled.get() ? ~0 : 0 );
    }
}

void
ModelLayer::fromConfig(const osgEarth::Config &conf)
{
    conf.getIfSet( "enabled", _enabled );
}