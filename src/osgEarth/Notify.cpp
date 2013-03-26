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
#include <osg/ApplicationUsage>
#include <osg/ref_ptr>
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

using namespace osgEarth;

static osg::ApplicationUsageProxy Notify_e0(osg::ApplicationUsage::ENVIRONMENTAL_VARIABLE, "OSGEARTH_NOTIFY_LEVEL <mode>", "FATAL | WARN | NOTICE | DEBUG_INFO | DEBUG_FP | DEBUG | INFO | ALWAYS");

namespace {
    typedef OpenThreads::ScopedLock<OpenThreads::Mutex> ScopedMutexLock;

    class ThreadNotifyData
    {
    public:
        inline ThreadNotifyData() 
            : notifyStream(new osgEarth::NotifyStream)
            {
                setNotifyHandler(osgEarth::getNotifyHandler());
            }
        inline ThreadNotifyData(const ThreadNotifyData & rhs)
            : notifyStream(rhs.notifyStream)
            {
                const_cast<ThreadNotifyData&>(rhs).notifyStream = NULL;
            }
        inline ~ThreadNotifyData()
            {
                delete notifyStream;
            }
        void setNotifyHandler(osg::NotifyHandler *handler)
        {
            osgEarth::NotifyStreamBuffer *buffer = static_cast<osgEarth::NotifyStreamBuffer*>(notifyStream->rdbuf());
            if (buffer) buffer->setNotifyHandler(handler);
        }

        osgEarth::NotifyStream * notifyStream;
    };
}

struct NotifySingleton
{
    NotifySingleton()
    {
        // _notifyLevel
        // =============

        _notifyLevel = osg::NOTICE; // Default value

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

            if(stringOSGNOTIFYLEVEL.find("ALWAYS")!=std::string::npos)          _notifyLevel=osg::ALWAYS;
            else if(stringOSGNOTIFYLEVEL.find("FATAL")!=std::string::npos)      _notifyLevel=osg::FATAL;
            else if(stringOSGNOTIFYLEVEL.find("WARN")!=std::string::npos)       _notifyLevel=osg::WARN;
            else if(stringOSGNOTIFYLEVEL.find("NOTICE")!=std::string::npos)     _notifyLevel=osg::NOTICE;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG_INFO")!=std::string::npos) _notifyLevel=osg::DEBUG_INFO;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG_FP")!=std::string::npos)   _notifyLevel=osg::DEBUG_FP;
            else if(stringOSGNOTIFYLEVEL.find("DEBUG")!=std::string::npos)      _notifyLevel=osg::DEBUG_INFO;
            else if(stringOSGNOTIFYLEVEL.find("INFO")!=std::string::npos)       _notifyLevel=osg::INFO;
            else std::cout << "Warning: invalid OSGEARTH_NOTIFY_LEVEL set ("<<stringOSGNOTIFYLEVEL<<")"<<std::endl;

        }

        // Setup standard notify handler
        _notifyHandler = new osg::StandardNotifyHandler;
    }
    
    typedef std::map<unsigned,ThreadNotifyData> ThreadNotifyDataMap;

    ThreadNotifyData& getThreadNotifyData()
    {
        ScopedMutexLock lock(_notifyStreamPerThreadMutex);
        return _notifyStreamPerThread[Threading::getCurrentThreadId()];
    }
    
    void setNotifyHandler(osg::NotifyHandler *handler)
    {
        _notifyHandler = handler;
        ScopedMutexLock lock(_notifyStreamPerThreadMutex);
        for(ThreadNotifyDataMap::iterator it = _notifyStreamPerThread.begin(); it != _notifyStreamPerThread.end(); it++)
        {
            ThreadNotifyData & thread = it->second;
            thread.setNotifyHandler(handler);
        }
    }

    osg::ref_ptr<osg::NotifyHandler> _notifyHandler;
    osg::NotifySeverity _notifyLevel;
    osgEarth::NullStream     _nullStream;
    OpenThreads::Mutex      _notifyStreamPerThreadMutex;
    ThreadNotifyDataMap     _notifyStreamPerThread;
};

static NotifySingleton& getNotifySingleton()
{
    static NotifySingleton s_NotifySingleton;
    return s_NotifySingleton;
}

bool osgEarth::initNotifyLevel()
{
    getNotifySingleton();
    return true;
}

// Use a proxy to force the initialization of the the NotifySingleton during static initialization
//OSG_INIT_SINGLETON_PROXY(NotifySingletonProxy, osgEarth::initNotifyLevel())
static struct NotifySingletonProxy{ NotifySingletonProxy() { osgEarth::initNotifyLevel(); } } s_NotifySingletonProxy;


void osgEarth::setNotifyLevel(osg::NotifySeverity severity)
{
	getNotifySingleton()._notifyLevel = severity;
}

osg::NotifySeverity osgEarth::getNotifyLevel()
{
    return getNotifySingleton()._notifyLevel;
}

void osgEarth::setNotifyHandler(osg::NotifyHandler *handler)
{
    getNotifySingleton().setNotifyHandler(handler);
}

osg::NotifyHandler* osgEarth::getNotifyHandler()
{
    return getNotifySingleton()._notifyHandler;
}

#ifndef OSG_NOTIFY_DISABLED
bool osgEarth::isNotifyEnabled( osg::NotifySeverity severity )
{
    return severity<=getNotifySingleton()._notifyLevel;
}
#endif

std::ostream& osgEarth::notify(const osg::NotifySeverity severity)
{
	if (osgEarth::isNotifyEnabled(severity))
	{
        ThreadNotifyData & data = getNotifySingleton().getThreadNotifyData();
		data.notifyStream->setCurrentSeverity(severity);
		return *data.notifyStream;
	}
	return getNotifySingleton()._nullStream;
}
