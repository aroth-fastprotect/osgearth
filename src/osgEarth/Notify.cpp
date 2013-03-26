/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarth/ThreadingUtils>
#include <osg/ref_ptr>
#include <osg/Notify>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cctype>
#include <iomanip>

using namespace std;
using namespace osgEarth;

namespace osgEarth {

class NullStreamBuffer : public std::streambuf
{
    private:
    std::streamsize xsputn(const std::streambuf::char_type *str, std::streamsize n)
        {
            return n;
        }
};

struct NullStream : public std::ostream
{
public:
    NullStream():
        std::ostream(new NullStreamBuffer)
    { _buffer = dynamic_cast<NullStreamBuffer *>(rdbuf()); }
        
    ~NullStream()
    {
        rdbuf(0);
        delete _buffer;
    }

protected:
    NullStreamBuffer* _buffer;
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

static bool s_osgEarthNeedNotifyInit = true;
static osg::NotifySeverity osgearth_g_NotifyLevel = osg::NOTICE;
static osgEarth::NullStream * g_NullStream = NULL;
static osg::NotifyHandler * g_NotifyHandler = NULL;

namespace {
    class ThreadNotifyData
    {
    public:
        inline ThreadNotifyData() 
            : notifyStream(new osgEarth::NotifyStream)
            {
                NotifyStreamBuffer *buffer = static_cast<NotifyStreamBuffer*>(notifyStream->rdbuf());
                if (buffer) buffer->setNotifyHandler(g_NotifyHandler);
            }
        inline ~ThreadNotifyData()
            {
                delete notifyStream;
            }

        osgEarth::NotifyStream * notifyStream;
    };
    
    ThreadNotifyData&
    getThreadNotifyData()
    {
        static Threading::PerThread<ThreadNotifyData> s_notifyPerThread;
        return s_notifyPerThread.get();
    }
}

void
osgEarth::setNotifyLevel(osg::NotifySeverity severity)
{
    if(s_osgEarthNeedNotifyInit) osgEarth::initNotifyLevel();
    osgearth_g_NotifyLevel = severity;
}

osg::NotifySeverity
osgEarth::getNotifyLevel()
{
    if(s_osgEarthNeedNotifyInit) osgEarth::initNotifyLevel();
    return osgearth_g_NotifyLevel;
}

bool
osgEarth::initNotifyLevel()
{
	if (!s_osgEarthNeedNotifyInit)
		return true;

	static osgEarth::NullStream s_NullStream;

	g_NullStream = &s_NullStream;

    // g_NotifyLevel
    // =============

    osgearth_g_NotifyLevel = osg::NOTICE; // Default value

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

        if(stringOSGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          osgearth_g_NotifyLevel=osg::ALWAYS;
        else if(stringOSGNOTIFYLEVEL.find("FATAL")!=std::string::npos)      osgearth_g_NotifyLevel=osg::FATAL;
        else if(stringOSGNOTIFYLEVEL.find("WARN")!=std::string::npos)       osgearth_g_NotifyLevel=osg::WARN;
        else if(stringOSGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     osgearth_g_NotifyLevel=osg::NOTICE;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) osgearth_g_NotifyLevel=osg::DEBUG_INFO;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   osgearth_g_NotifyLevel=osg::DEBUG_FP;
        else if(stringOSGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      osgearth_g_NotifyLevel=osg::DEBUG_INFO;
        else if(stringOSGNOTIFYLEVEL.find("INFO")!=std::string::npos)       osgearth_g_NotifyLevel=osg::INFO;
        else std::cout << "Warning: invalid OSG_NOTIFY_LEVEL set ("<<stringOSGNOTIFYLEVEL<<")"<<std::endl;
 
    }

	s_osgEarthNeedNotifyInit = false;

    return true;

}

#ifndef OSG_NOTIFY_DISABLED
bool osgEarth::isNotifyEnabled( osg::NotifySeverity severity )
{
	if (s_osgEarthNeedNotifyInit) osgEarth::initNotifyLevel();
    return severity<=getNotifyLevel();
}
#endif

void osgEarth::setNotifyHandler(osg::NotifyHandler *handler)
{
    g_NotifyHandler = handler;
}

osg::NotifyHandler* osgEarth::getNotifyHandler()
{
	if (s_osgEarthNeedNotifyInit) osgEarth::initNotifyLevel();
    return g_NotifyHandler;
}


std::ostream& osgEarth::notify(const osg::NotifySeverity severity)
{
	if (s_osgEarthNeedNotifyInit) osgEarth::initNotifyLevel();

	if (osgEarth::isNotifyEnabled(severity))
	{
        ThreadNotifyData & data = getThreadNotifyData();
		data.notifyStream->setCurrentSeverity(severity);
		return *data.notifyStream;
	}
	return *g_NullStream;
}
