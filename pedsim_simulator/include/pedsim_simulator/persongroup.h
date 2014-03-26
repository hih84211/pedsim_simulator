/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#ifndef PERSONGROUP_H
#define PERSONGROUP_H

#include <pedsim_simulator/utilities.h>
#include <pedsim_simulator/agent.h>

/// -----------------------------------------------------------------
/// \class PersonGroup
/// \brief Group of people / agents
/// \details Model for a group of people \f$(1 - N)\f$
/// -----------------------------------------------------------------
class PersonGroup
{
public:

    PersonGroup ();
    PersonGroup ( const std::list<Agent*>& agents );
    virtual ~PersonGroup ();

    std::list<Agent*>& getMembers ();
    const std::list<Agent*>& getMembers () const;
    bool addMember ( Agent* agent );
    bool removeMember ( Agent* agent );
    bool setMembers ( const std::list<Agent*>& agents );
    bool isEmpty () const;
    bool isMember ( Agent* agent );
    size_t memberCount () const;
	
	void computeGroupForces ();
	Ped::Tvector gazeForce();
	Ped::Tvector coherenceForce();
	Ped::Tvector repulsionForce();

private:

    std::list<Agent*> members_;
	
	Ped::Tvector force_gaze_;
	Ped::Tvector force_coherence_;
	Ped::Tvector force_repulsion_;
};


/// helpful typedefs
typedef boost::shared_ptr<PersonGroup> PersonGroupPtr;
typedef boost::shared_ptr<PersonGroup const> PersonGroupConstPtr;



#endif // PERSONGROUP_H
