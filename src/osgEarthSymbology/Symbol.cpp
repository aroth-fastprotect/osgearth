/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthSymbology/Symbol>

using namespace osgEarth;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

Symbol::Symbol( const Config& conf )
{
    _uriContext = URIContext(conf.referrer());
}

bool
Symbol::match(const std::string& s, const char* reservedWord)
{
    if ( s.compare(reservedWord) == 0 ) return true;
    //if ( s == reservedWord ) return true;
    std::string temp1 = toLower(s), temp2 = toLower(reservedWord);
    replaceIn(temp1, "_", "-");
    replaceIn(temp2, "_", "-");
    return temp1.compare(temp2) == 0;
}
