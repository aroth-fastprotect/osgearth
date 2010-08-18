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
#include <osgEarth/Notify>

using namespace osgEarth;

/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/
#include <osgEarth/Notify>
#include <osg/ref_ptr>
#include <osg/Notify>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctype.h>

using namespace std;

namespace osgEarth {

class NullStreamBuffer : public std::streambuf
{
    private:
    
        virtual streamsize xsputn (const char_type*, streamsize n)
        {
            return n;
        }
};

struct NullStream : public std::ostream
{
    NullStream():
        std::ostream(new NullStreamBuffer) {}
        
    virtual ~NullStream()
    {
        delete rdbuf();
        rdbuf(0);
    }
};

/** Stream buffer calling notify handler when buffer is synchronized (usually on std::endl).
 * Stream stores last notification severity to pass it to handler call.
 */
struct NotifyStreamBuffer : public std::stringbuf
{
    NotifyStreamBuffer() : _severity(osg::NOTICE)
    {
    }

    void setNotifyHandler(osg::NotifyHandler *handler) { _handler = handler; }
    osg::NotifyHandler *getNotifyHandler() const { return _handler.get(); }

    /** Sets severity for next call of notify handler */
    void setCurrentSeverity(osg::NotifySeverity severity) { _severity = severity; }
    osg::NotifySeverity getCurrentSeverity() const { return _severity; }

private:

    int sync()
    {
        sputc(0); // string termination
        if (_handler.valid())
            _handler->notify(_severity, pbase());
        pubseekpos(0, std::ios_base::out); // or str(std::string())
        return 0;
    }

    osg::ref_ptr<osg::NotifyHandler> _handler;
    osg::NotifySeverity _severity;
};

struct NotifyStream : public std::ostream
{
public:
    NotifyStream():
        std::ostream(new NotifyStreamBuffer)
    { _buffer = dynamic_cast<NotifyStreamBuffer *>(rdbuf()); }

    void setCurrentSeverity(osg::NotifySeverity severity)
    {
        _buffer->setCurrentSeverity(severity);
    }

    osg::NotifySeverity getCurrentSeverity() const
    {
        return _buffer->getCurrentSeverity();
    }
        
    ~NotifyStream()
    {
        rdbuf(0);
        delete _buffer;
    }

protected:
    NotifyStreamBuffer* _buffer;
};

} // namespace osgEarth

static bool s_NeedNotifyInit = true;
static osg::NotifySeverity g_NotifyLevel = osg::NOTICE;
static osgEarth::NullStream *g_NullStream;
static osgEarth::NotifyStream *g_NotifyStream;

void
osgEarth::setNotifyLevel(osg::NotifySeverity severity)
{
    osgEarth::initNotifyLevel();
    g_NotifyLevel = severity;
}

osg::NotifySeverity
osgEarth::getNotifyLevel()
{
    osgEarth::initNotifyLevel();
    return g_NotifyLevel;
}

bool
osgEarth::initNotifyLevel()
{
	static osgEarth::NullStream s_NullStream;
	static osgEarth::NotifyStream s_NotifyStream;

	g_NullStream = &s_NullStream;
	g_NotifyStream = &s_NotifyStream;

	// g_NotifyLevel
	// =============

	g_NotifyLevel = osg::NOTICE; // Default value

	char* OSGNOTIFYLEVEL=getenv("OSGEARTH_NOTIFY_LEVEL");
	if (!OSGNOTIFYLEVEL) OSGNOTIFYLEVEL=getenv("OSGEARTHNOTIFYLEVEL");
	if(OSGNOTIFYLEVEL)
	{

		std::string stringOSGNOTIFYLEVEL(OSGNOTIFYLEVEL);

		// Convert to upper case
		for(std::string::iterator i=stringOSGNOTIFYLEVEL.begin();
			i!=stringOSGNOTIFYLEVEL.end();
			++i)
		{
			*i=toupper(*i);
		}

		if(stringOSGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          g_NotifyLevel=osg::ALWAYS;
		else if(stringOSGNOTIFYLEVEL.find("FATAL")!=std::string::npos)      g_NotifyLevel=osg::FATAL;
		else if(stringOSGNOTIFYLEVEL.find("WARN")!=std::string::npos)       g_NotifyLevel=osg::WARN;
		else if(stringOSGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     g_NotifyLevel=osg::NOTICE;
		else if(stringOSGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) g_NotifyLevel=osg::DEBUG_INFO;
		else if(stringOSGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   g_NotifyLevel=osg::DEBUG_FP;
		else if(stringOSGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      g_NotifyLevel=osg::DEBUG_INFO;
		else if(stringOSGNOTIFYLEVEL.find("INFO")!=std::string::npos)       g_NotifyLevel=osg::INFO;
		else std::cout << "Warning: invalid OSG_NOTIFY_LEVEL set ("<<stringOSGNOTIFYLEVEL<<")"<<std::endl;

	}

	// Setup standard notify handler
	osgEarth::NotifyStreamBuffer *buffer = dynamic_cast<osgEarth::NotifyStreamBuffer *>(g_NotifyStream->rdbuf());
	if (buffer && !buffer->getNotifyHandler())
		buffer->setNotifyHandler(new osg::StandardNotifyHandler);

	s_NeedNotifyInit = false;

	return true;
}

bool osgEarth::isNotifyEnabled( osg::NotifySeverity severity )
{
	if (s_NeedNotifyInit) osgEarth::initNotifyLevel();
	return severity<=g_NotifyLevel;
}


void osgEarth::setNotifyHandler(osg::NotifyHandler *handler)
{
	osgEarth::NotifyStreamBuffer *buffer = static_cast<osgEarth::NotifyStreamBuffer *>(g_NotifyStream->rdbuf());
	if (buffer)
		buffer->setNotifyHandler(handler);
}

osg::NotifyHandler* osgEarth::getNotifyHandler()
{
	if (s_NeedNotifyInit) osg::initNotifyLevel();
	osgEarth::NotifyStreamBuffer *buffer = static_cast<osgEarth::NotifyStreamBuffer *>(g_NotifyStream->rdbuf());
	return buffer ? buffer->getNotifyHandler() : 0;
}


std::ostream& osgEarth::notify(const osg::NotifySeverity severity)
{
	if (s_NeedNotifyInit) osgEarth::initNotifyLevel();

	if (osgEarth::isNotifyEnabled(severity))
	{
		g_NotifyStream->setCurrentSeverity(severity);
		return *g_NotifyStream;
	}
	return *g_NullStream;
}
/*
std::ostream&
osgEarth::notify(const osg::NotifySeverity severity)
{
    // set up global notify null stream for inline notify
    static NullStream s_NotifyNulStream;

    static bool initialized = false;
    if (!initialized) 
    {
        std::cerr<<""; // dummy op to force construction of cerr, before a reference is passed back to calling code.
        std::cout<<""; // dummy op to force construction of cout, before a reference is passed back to calling code.
        initialized = osgEarth::initNotifyLevel();
    }

	if (osg::isNotifyEnabled(severity))
	{
		g_NotifyStream->setCurrentSeverity(severity);
		return *g_NotifyStream;
	}
	return s_NotifyNulStream;

    if (severity<=osgearth_g_NotifyLevel)
    {
        if (severity<=osg::WARN) return std::cerr;
        else return std::cout;
    }
    return s_NotifyNulStream;
}
*/
