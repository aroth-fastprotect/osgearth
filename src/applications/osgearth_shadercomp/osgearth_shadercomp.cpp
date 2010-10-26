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
#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarth/ShaderComposition>
#include <osgEarth/Registry>

osg::StateAttribute* createHaze();


int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);
    osgViewer::Viewer viewer(arguments);
    viewer.setCameraManipulator( new osgEarthUtil::EarthManipulator() );

    osg::Node* earthNode = osgDB::readNodeFiles( arguments );
    if (!earthNode)
    {
        OE_WARN << "Unable to load earth model." << std::endl;
        return -1;
    }

    osg::Group* root = new osg::Group();
    root->addChild( earthNode );
    viewer.setSceneData( root );

    // inject the haze shader components:
    root->getOrCreateStateSet()->setAttributeAndModes( createHaze(), osg::StateAttribute::ON );

    // add some stock OSG handlers:
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgViewer::ThreadingHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    return viewer.run();
}

//-------------------------------------------------------------------------

static char s_hazeVertShader[] =
    "varying vec3 v_pos; \n"
    "void osgearth_vert_postprocess(in vec3 position, in vec3 normal) \n"
    "{ \n"
    "    v_pos = vec3(gl_ModelViewMatrix * gl_Vertex); \n"
    "} \n";

static char s_hazeFragShader[] =
    "varying vec3 v_pos; \n"
    "void osgearth_frag_postprocess(inout vec4 color) \n"
    "{ \n"
    "    float dist = clamp( length(v_pos)/10000000.0, 0, 0.75 ); \n"
    "    color = mix(color, vec4(0.5, 0.5, 0.5, 1.0), dist); \n"
    "} \n";


osg::StateAttribute*
createHaze()
{
    osgEarth::ShaderFactory* fact = osgEarth::Registry::instance()->getShaderFactory();
    fact->setUseInjectionPoint( osgEarth::ShaderFactory::INJECT_POST_VERTEX );
    fact->setUseInjectionPoint( osgEarth::ShaderFactory::INJECT_POST_FRAGMENT );

    osgEarth::VirtualProgram* vp = new osgEarth::VirtualProgram();
    vp->setShader( "osgearth_vert_postprocess", new osg::Shader( osg::Shader::VERTEX, s_hazeVertShader ) );
    vp->setShader( "osgearth_frag_postprocess", new osg::Shader( osg::Shader::FRAGMENT, s_hazeFragShader ) );

    return vp;
}
