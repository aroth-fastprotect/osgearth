/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthSymbology/StyleSelector>

#define LC "[StyleSelector] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

StyleSelector::StyleSelector( const Config& conf )
{
    mergeConfig( conf );
}

std::string
StyleSelector::getSelectedStyleName() const 
{
    return _styleName.isSet() ? *_styleName : _name;
}

void
StyleSelector::mergeConfig( const Config& conf )
{
    _name = conf.value( "name" );
    conf.getIfSet   ( "style",        _styleName );
    conf.getIfSet   ( "class",        _styleName ); // alias
    conf.getObjIfSet( "style_expr",   _styleExpression ); 
    conf.getObjIfSet( "class_expr",   _styleExpression ); // alias
    conf.getObjIfSet( "query",        _query );
}

Config
StyleSelector::getConfig() const
{
    Config conf( "selector" );
    conf.add        ( "name",         _name );
    conf.addIfSet   ( "style",        _styleName );
    conf.addObjIfSet( "style_expr",   _styleExpression );
    conf.addObjIfSet( "query",        _query );
    return conf;
}
