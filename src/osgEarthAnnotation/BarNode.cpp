/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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

#include <osgEarthAnnotation/BarNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarth/MapNode>
#include <osgEarth/ScreenSpaceLayout>
#include <osg/Geometry>
#include <cassert>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

namespace {

class ColorGradient
{
private:
    struct ColorPoint  // Internal class used to store colors at different points in the gradient.
    {
        osg::Vec4 color;      // Red, green and blue values of our color.
        float value;        // Position of our color along the gradient (between 0 and 1).
        ColorPoint(const osg::Vec4 & c, float v)
            : color(c), value(v) {}
    };
    std::vector<ColorPoint> color;      // An array of color points in ascending value.

public:
    //-- Default constructor:
    ColorGradient() { createDefaultHeatMapGradient(); }
    ColorGradient(const osg::Vec4 & a, const osg::Vec4& b)
    {
        create(a, b);
    }

    //-- Inserts a new color point into its correct position:
    void addColorPoint(const osg::Vec4 & c, float value)
    {
        for (unsigned i = 0; i < color.size(); i++) {
            if (value < color[i].value) {
                color.insert(color.begin() + i, ColorPoint(c, value));
                return;
            }
        }
        color.push_back(ColorPoint(c, value));
    }

    //-- Inserts a new color point into its correct position:
    void clearGradient() { color.clear(); }

    //-- Places a 5 color heapmap gradient into the "color" vector:
    void createDefaultHeatMapGradient()
    {
        color.clear();
        color.push_back(ColorPoint(osg::Vec4(0, 0, 1, 1.0f), 0.0f));      // Blue.
        color.push_back(ColorPoint(osg::Vec4(0, 1, 1, 1.0f), 0.25f));     // Cyan.
        color.push_back(ColorPoint(osg::Vec4(0, 1, 0, 1.0f), 0.5f));      // Green.
        color.push_back(ColorPoint(osg::Vec4(1, 1, 0, 1.0f), 0.75f));     // Yellow.
        color.push_back(ColorPoint(osg::Vec4(1, 0, 0, 1.0f), 1.0f));      // Red.
    }

    void create(const osg::Vec4& a, const osg::Vec4& b)
    {
        color.clear();
        color.push_back(ColorPoint(a, 0.0f));      // Blue.
        color.push_back(ColorPoint(b, 1.0f));      // Red.
    }

    //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
    //-- values representing that position in the gradient.
    void getColorAtValue(const float value, osg::Vec4 & out)
    {
        if (color.size() == 0)
            return;

        for (unsigned i = 0; i < color.size(); i++)
        {
            ColorPoint& currC = color[i];
            if (value < currC.value)
            {
                ColorPoint& prevC = color[std::max(0, (int)i - 1)];
                float valueDiff = (prevC.value - currC.value);
                float fractBetween = (valueDiff == 0) ? 0 : (value - currC.value) / valueDiff;
                out.x() = (prevC.color.x() - currC.color.x()) * fractBetween + currC.color.x();
                out.y() = (prevC.color.y() - currC.color.y()) * fractBetween + currC.color.y();
                out.z() = (prevC.color.z() - currC.color.z()) * fractBetween + currC.color.z();
                out.w() = (prevC.color.w() - currC.color.w()) * fractBetween + currC.color.w();
                return;
            }
        }
        out = color.back().color;
        return;
    }
};

} // namespace

BarNode::BarNode() :
    GeoPositionNode ()
{
    construct();
}

void
BarNode::construct()
{
    setName("Bar");
}

void
BarNode::addValue(const Value & v)
{
    _values.push_back(v);
}

void
BarNode::addValue(const float & value, const float & totalHeight, const Fill & minimumValueColor, const Fill & maximumValueColor)
{
    Value v;
    v.value = value;
    v.totalHeight = totalHeight;
    v.minimumValueColor = minimumValueColor;
    v.maximumValueColor = maximumValueColor;
    _values.push_back(v);
}

void
BarNode::clear()
{
    _values.clear();
}

void
BarNode::setStyle( const Style& style )
{
    _style = style;

    compile();
}

void BarNode::compile()
{
    applyStyle( _style );

    buildGeometry();
    dirty();
}

void
BarNode::applyStyle(const Style& style)
{
    GeoPositionNode::applyStyle(style);
    const BarSymbol* bar = style.get<BarSymbol>();
    if (bar)
    {
        float width = bar->width().value().eval();
        _width = width;

        for(BarSymbol::ValueList::const_iterator it = bar->values().begin(); it != bar->values().end(); ++it)
        {
            const BarSymbol::Value& entry = *it;

            float valueScale = entry.valueScale().value().eval();
            float value = entry.value().value().eval() * valueScale;
            Color minColor = entry.minimumColor().value().color();
            Color maxColor = entry.maximumColor().value().color();

            float minVal = entry.minimumValue().value().eval() * valueScale;
            float maxVal = entry.maximumValue().value().eval() * valueScale;

            float range = maxVal - minVal;
            float percent = (value - minVal) / range;
            addValue(percent, value, minColor, maxColor);
        }
    }
    else {
        clear();
    }
}

void
BarNode::updateLayoutData()
{
    if (!_dataLayout.valid())
    {
        _dataLayout = new ScreenSpaceLayoutData();
    }
    _geom->setUserData(_dataLayout.get());

    _dataLayout->setPriority(getPriority());

}

void
BarNode::buildGeometry()
{
    _geom = new osg::Geometry;
    float overallTotalHeight = 0;
    float barHeight = 0;
    float barWidth = _width.getValue();
    const unsigned num_vertices_per_value_set = 8;
    const unsigned num_indicies_per_value_set = 14;
    unsigned numVertices = _values.size() * num_vertices_per_value_set;
    unsigned numIndicies = _values.size() * num_indicies_per_value_set;
    osg::Vec3Array* vertices = new osg::Vec3Array(numVertices);
    osg::Vec4Array * colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX, numVertices);
    osg::DrawElementsUByte* indices = new osg::DrawElementsUByte(osg::PrimitiveSet::TRIANGLE_STRIP, numIndicies);

    _geom->insertPrimitiveSet(0, indices);

    indices->resize(numIndicies);
    indices->setNumInstances(numIndicies);

    static unsigned char elements[] = {
        3, 2, 6, 7, 4, 2, 0,
        3, 1, 6, 5, 4, 1, 0
    };
    assert(sizeof(elements) / sizeof(elements[0]) == num_indicies_per_value_set);
    unsigned vertexIndex = 0, indiciesIndex = 0;
    for(ValueList::const_iterator it = _values.begin(); it != _values.end(); 
        ++it, vertexIndex += num_vertices_per_value_set, indiciesIndex += num_indicies_per_value_set)
    {
        const Value & v = *it;
        float bottomHeight = barHeight;
        overallTotalHeight += v.totalHeight;
        float valueHeight = v.value * v.totalHeight;
        barHeight += valueHeight;
        float topHeight = barHeight;

        (*vertices)[vertexIndex + 0].set(barWidth, barWidth, topHeight);
        (*vertices)[vertexIndex + 1].set(-barWidth, barWidth, topHeight);
        (*vertices)[vertexIndex + 2].set(barWidth, barWidth, bottomHeight);
        (*vertices)[vertexIndex + 3].set(-barWidth, barWidth, bottomHeight);
        (*vertices)[vertexIndex + 4].set(barWidth, -barWidth, topHeight);
        (*vertices)[vertexIndex + 5].set(-barWidth, -barWidth, topHeight);
        (*vertices)[vertexIndex + 6].set(-barWidth, -barWidth, bottomHeight);
        (*vertices)[vertexIndex + 7].set(barWidth, -barWidth, bottomHeight);

        for (unsigned int c = 0; c < num_indicies_per_value_set; ++c)
            (*indices)[indiciesIndex + c] = vertexIndex + elements[c];

        osg::Vec4 bottomColor = v.minimumValueColor.color();
        ColorGradient gradient(bottomColor, v.maximumValueColor.color());
        osg::Vec4 valueColor;
        gradient.getColorAtValue(v.value, valueColor);

        (*colors)[vertexIndex + 0] = valueColor;
        (*colors)[vertexIndex + 1] = valueColor;
        (*colors)[vertexIndex + 2] = bottomColor;
        (*colors)[vertexIndex + 3] = bottomColor;
        (*colors)[vertexIndex + 4] = valueColor;
        (*colors)[vertexIndex + 5] = valueColor;
        (*colors)[vertexIndex + 6] = bottomColor;
        (*colors)[vertexIndex + 7] = bottomColor;
    }
    _geom->setVertexArray(vertices);
    _geom->setColorArray(colors);
    getPositionAttitudeTransform()->addChild(_geom.get());

    setDefaultLighting(false);

    setPriority(getPriority());

    if (_dynamic)
        setDynamic(_dynamic);

    updateLayoutData();
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( bar, osgEarth::Annotation::BarNode );


BarNode::BarNode(const Config&         conf,
                 const osgDB::Options* dbOptions) :
    GeoPositionNode (conf, dbOptions)
{
    construct();

    buildGeometry();
}

Config
BarNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "bar";

    return conf;
}
