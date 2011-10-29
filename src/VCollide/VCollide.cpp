/************************************************************************\

  Copyright 1997 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following three paragraphs appear in all copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL
  HILL BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
  INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS,
  ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
  EVEN IF THE UNIVERSITY OF NORTH CAROLINA HAVE BEEN ADVISED OF
  THE POSSIBILITY OF SUCH DAMAGES.


  Permission to use, copy, modify and distribute this software
  and its documentation for educational, research and non-profit
  purposes, without fee, and without a written agreement is
  hereby granted, provided that the above copyright notice and
  the following three paragraphs appear in all copies.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
  BASIS, AND THE UNIVERSITY OF NORTH CAROLINA HAS NO OBLIGATION
  TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
  MODIFICATIONS.


   ---------------------------------
  |Please send all BUG REPORTS to:  |
  |                                 |
  |   geom@cs.unc.edu               |
  |                                 |
   ---------------------------------


  The authors may be contacted via:

  US Mail:  A. Pattekar/J. Cohen/T. Hudson/S. Gottschalk/M. Lin/D. Manocha
            Department of Computer Science
            Sitterson Hall, CB #3175
            University of N. Carolina
            Chapel Hill, NC 27599-3175

  Phone:    (919)962-1749

  EMail:    geom@cs.unc.edu

\************************************************************************/

/************************************************************************\
Filename: VCollide.C
--
Description: This file implements the member functions of the class VCollide.
             The class VCollide is simply a user interface and and is
             designed for data hiding. Hence, these member functions
             simply return make calls to the corresponding member
             functions of the class VCInternal.
\************************************************************************/

#include "VCollide.h"
#include "VInternal.H"


VCollide::VCollide()
{
  vcint = new VCInternal;
}

VCollide::~VCollide()
{
  delete vcint;
}

int VCollide::NewObject(int *id)    //create a new object in the database.
{
  return vcint->NewObject(id);
}

int VCollide::AddTri(double v1[], double v2[], double v3[], int tri_id)  //insert
                                                                         //the geometry
{
  return vcint->AddTri(v1, v2, v3, tri_id);
}


int VCollide::EndObject(void) //tell VCollide that inserting the
                              //geometry is complete.
{
  return vcint->EndObject();
}

int VCollide::UpdateTrans(int id, double t[][4])
  //update the transformation matrix of the object.
{
  return vcint->UpdateTrans(id, t);
}


int VCollide::ActivateObject(int id)
{
  return vcint->ActivateObject(id);
}

int VCollide::DeactivateObject(int id)
{
  return vcint->DeactivateObject(id);
}


int VCollide::ActivatePair(int id1, int id2)  //activate the pair.
{
  return vcint->ActivatePair(id1, id2);
}

int VCollide::DeactivatePair(int id1, int id2)//deactivate the pair.
{
  return vcint->DeactivatePair(id1, id2);
}


int VCollide::DeleteObject(int id)  //delete the object from the database.
{
  return vcint->DeleteObject(id);
}


int VCollide::Collide(VCReport *report, int flag)
    //perform collision detection and return a report.
{
  return vcint->Collide(report, flag);
}
