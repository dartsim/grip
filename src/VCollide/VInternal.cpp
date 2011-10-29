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
Filename: VInternal.C
--
Description: This file implements the member functions of the classes
	     VInternal and VCReport.

\************************************************************************/



#include <iostream>
#include <string.h>     //for memset and memcpy. ** edited to string.h from string by jon
#include "VInternal.H"

const int DEFAULT_SIZE=10; //some arbitrary default size for "vc_objects" array.

//error codes returned by VCollide API.
//these are multiply defined in the file VCollide.H
//so, any changes to these need to be reflected in all three places.
const int VC_ERR_INVALID_ID            = -4; //invalid id was passed to the
                                             //routine.
const int VC_ERR_EMPTY_OBJECT          = -3; //EndObject called without adding
                                             //adding any triangles.
const int VC_ERR_CALL_OUT_OF_SEQUENCE  = -2; //calls out of sequence.
const int VC_ERR                       = -1; //some other error.
const int VC_OK                        =  1; //No error.

// these are multiply defined in VCollide.H.
const int VC_ALL_CONTACTS  = 1;  // want RAPID to return all contact
                                 // information between each object pair.
const int VC_FIRST_CONTACT = 2;  // want RAPID to return only first contact
                                 // between each object pair.


// used by VCReport
class VCObjPair
{
    friend class VCInternal;
    friend class VCReport;

public:
    VCObjPair() { num_tri_pairs = 0; tri_pair = 0; };
    ~VCObjPair() { delete [] tri_pair; };

private:
    int id1, id2;  // ids of the intersecting objects.

    int num_tri_pairs;  // total number of pairs of intersecting triangles
                        // between the 2 objects.

    TriPair *tri_pair;  // an array of the triangle pairs.
			// TriPair is declared in PairData.H.
};


// multiply defined in VCollide.H.
#ifndef _VCREPORT
#define _VCREPORT

class VCReport
{
    friend class VCInternal;

private:
    int num_obj_pairs;    // number of intersecting object-pairs.
    VCObjPair *obj_pair;  // an array of the object pairs.

public:
    VCReport();
    ~VCReport();
    int numObjPairs();
    int obj1ID( int obj_pair_num );
    int obj2ID( int obj_pair_num );
    int numTriPairs( int obj_pair_num );
    int tri1ID( int obj_pair_num, int tri_pair_num );
    int tri2ID( int obj_pair_num, int tri_pair_num );
};

#endif


VCReport::VCReport()
{
    num_obj_pairs = 0;
    obj_pair = 0;
}


VCReport::~VCReport()
{
    delete [] obj_pair;
}


int VCReport::numObjPairs()
{
    return num_obj_pairs;
}


int VCReport::obj1ID( int obj_pair_num )
{
    return obj_pair[obj_pair_num].id1;
}


int VCReport::obj2ID( int obj_pair_num )
{
    return obj_pair[obj_pair_num].id2;
}


int VCReport::numTriPairs( int obj_pair_num )
{
    return obj_pair[obj_pair_num].num_tri_pairs;
}


int VCReport::tri1ID( int obj_pair_num, int tri_pair_num )
{
    return obj_pair[obj_pair_num].tri_pair[tri_pair_num].tid1;
}


int VCReport::tri2ID( int obj_pair_num, int tri_pair_num )
{
    return obj_pair[obj_pair_num].tri_pair[tri_pair_num].tid2;
}





VCInternal::VCInternal()
{
  state = VCstate_default;
  next_id = 0;

  size = DEFAULT_SIZE;                   //set the size of the array.
  vc_objects = new VCObject*[size]; //allocate the array.
  int i;
  for (i=0; i<size; i++)
    vc_objects[i] = NULL;

  disabled.Clear();  //to begin with, no pairs are disabled.
}


VCInternal::~VCInternal()
{

  //deallocate the memory.
  int i;
  for (i=0; i<size; i++)
    {
      if (vc_objects[i])
        {
          delete vc_objects[i]->b;
          delete vc_objects[i];
        }
    }
  delete [] vc_objects;
}



int VCInternal::NewObject(int *id) //create a new object in the database.
{
  //check if we are in the correct state.
  if (state != VCstate_default)
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }
  state = VCstate_newObject; //set the new state.

  //increase the size of the "vc_objects" array if required.
  if (next_id >= size)
    {
      int newsize = (next_id >= 2*size) ? (next_id+1) : 2*size;
      VCObject **temp = new VCObject*[newsize];
      int i;
      for (i=0; i<size; i++)
        temp[i] = vc_objects[i];
      for (i=size; i<newsize; i++)
        temp[i] = NULL;
      delete [] vc_objects;
      vc_objects = temp;
      size = newsize;

    }

  //allocate a new object.
  vc_objects[next_id] = new VCObject;
  if (vc_objects[next_id] == NULL)
    return VC_ERR;

  *id = next_id;  //for returning the id generated by VCollide.
  current_id = next_id;
  vc_objects[next_id]->id = next_id;
  vc_objects[next_id]->b = new RAPID_model;
  vc_objects[next_id]->b->BeginModel();
  vc_objects[next_id]->activation_state = 1;
  next_id++;

  return VC_OK;
}

int VCInternal::AddTri(double v1[], double v2[], double v3[], int tri_id)
{                     //add geometry to the newly created object.

  //check whether we are in the correct state.
  if ( (state != VCstate_newObject) && (state != VCstate_addTri) )
    {
      //cout<<"state is: "<<state<<"\n";
      return VC_ERR_CALL_OUT_OF_SEQUENCE;
    }

  state=VCstate_addTri; //set the new state.

  vc_objects[current_id]->b->AddTri(v1, v2, v3, tri_id);  //add triangle.
  return VC_OK;

}


int VCInternal::EndObject(void)
{   //tells VCollide that inputting the geometry is complete.

  //check whether we are in the correct state.
  if (state == VCstate_newObject)
    return VC_ERR_EMPTY_OBJECT;
  else if (state != VCstate_addTri)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  state = VCstate_default;  //set the new state.


  //add the object to the NBody database.
  nbody.AddObject(current_id, vc_objects[current_id]->b);

  //now, have RAPID build the OBB tree.
  vc_objects[current_id]->b->EndModel();

  //initialize the transformation matrix to identity.
  //doing it the following way is probably faster, since the
  //compiler can use lower level memory calls for initialization.
  memset( ( (void *)vc_objects[current_id]->trans), 0, 16*sizeof(double) );
  vc_objects[current_id]->trans[0][0] = 1.0;
  vc_objects[current_id]->trans[1][1] = 1.0;
  vc_objects[current_id]->trans[2][2] = 1.0;
  vc_objects[current_id]->trans[3][3] = 1.0;

  return VC_OK;

}

int VCInternal::UpdateTrans(int id, double t[][4])
{             //update the transformation matrix of the object.
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;


  VCObject *current;

  if (id >= size)  //invalid id.
    {
      //cerr<<"VCInternal::update_trans - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::update_trans - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      current = vc_objects[id];
    }

  //update the private copy of the transformation matrix.
  memcpy((void *)current->trans, (void *)t, 16*sizeof(double));

  //have the nbody database update itself appropriately.
  nbody.UpdateTrans(current->id, t);

  return VC_OK;

}


int VCInternal::ActivateObject(int id)
{  //activate an object for collision detection.

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  if (id >= size)  //invalid id.
    {
      //cerr<<"VCInternal::activate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::activate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      vc_objects[id]->activation_state = 1;
    }
  return VC_OK;
}

int VCInternal::DeactivateObject(int id)
{  //deactivate an object from collision detection.

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  if (id >= size)  //invalid id.
    {
      //cerr<<"VCInternal::deactivate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else if (vc_objects[id] == NULL)  //invalid id.
    {
      //cerr<<"VCInternal::deactivate - no object with id = "<<id<<"\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      vc_objects[id]->activation_state = 0;
    }
  return VC_OK;
}


int VCInternal::ActivatePair(int id1, int id2)
{

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  if ( (id1 >= size) || (id2 >= size) )  //invalid id.
    {
      //cerr<<"VCInternal::activate_pair - invalid id for activation\n";
      return VC_ERR_INVALID_ID;
    }
  else if ( (vc_objects[id1] == NULL) || (vc_objects[id2] == NULL) )//invalid id.
    {
      //cerr<<"VCInternal::activate_pair - invalid id for activation\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      disabled.DelPair(id1, id2);
      return VC_OK;
    }
}

int VCInternal::DeactivatePair(int id1, int id2)
{

  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  if ( (id1 >= size) || (id2 >= size) )  //invalid id.
    {
      //cerr<<"VCInternal::deactivate_pair - invalid id for deactivation\n";
      return VC_ERR_INVALID_ID;
    }
  else if ( (vc_objects[id1] == NULL) || (vc_objects[id2] == NULL) )//invalid id.
    {
      //cerr<<"VCInternal::deactivate_pair - invalid id for deactivation\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      if (id1!=id2)
        disabled.AddPair(id1, id2);

      return VC_OK;
    }
}


int VCInternal::DeleteObject(int id) //delete an object from the database.
{
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;


  if (id >= size) //invalid id.
    {
      //cerr<<"VCollide::delete_object - object with id = "<<id<<" does not exist\n";
      return VC_ERR_INVALID_ID;
    }

  if (vc_objects[id] == NULL) //invalid id.
    {
      //cerr<<"VCollide::delete_object - object with id = "<<id<<" does not exist\n";
      return VC_ERR_INVALID_ID;
    }
  else
    {
      delete vc_objects[id]->b; //delete the RAPID box.
      delete vc_objects[id];    //delete the object.
      vc_objects[id] = NULL;

      disabled.DelPairsInvolvingId(id);

      nbody.DeleteObject(id); //delete the object from the nbody database.
      return VC_OK;
    }

}



int VCInternal::Collide(VCReport *report, int flag)
    //perform collision detection and return a report.
{
  //check whether we are in the right state.
  if (state != VCstate_default)
    return VC_ERR_CALL_OUT_OF_SEQUENCE;

  //Clear the results from earlier collision tests.
  report_data.Clear();

  //Simultaneously traverse the "overlapping_pairs" database and the
  //"disabled_pairs" database and make calls to the RAPID collision
  //detection routine where required.
  int i;
  for (i=0; i<nbody.overlapping_pairs.size; i++)
    {
      Elem *curr_ovrlp = nbody.overlapping_pairs.arr[i];

      Elem *curr_disabled;
      if (i<disabled.size)
        curr_disabled = disabled.arr[i];
      else
        curr_disabled = NULL;

      if (curr_ovrlp != NULL)
        {
          if (vc_objects[i]->activation_state == 1)
            {
              while (curr_ovrlp != NULL)
                {
                  while (curr_disabled != NULL)
                    {
                      if (curr_disabled->id < curr_ovrlp->id)
                        curr_disabled = curr_disabled->next;
                      else
                        break;
                    }

                  if (curr_disabled != NULL)
                    {
                      if (curr_disabled->id > curr_ovrlp->id)
                        {
                          if (vc_objects[curr_ovrlp->id]->activation_state == 1)
                            {
                              //now, we need to call the RAPID collision detection routine.

                              double R1[3][3], T1[3], R2[3][3], T2[3];

                              //set up the rotation and translation matrices as
                              //required by RAPID for the first object.
                              R1[0][0] = vc_objects[i]->trans[0][0];
                              R1[0][1] = vc_objects[i]->trans[0][1];
                              R1[0][2] = vc_objects[i]->trans[0][2];
                              R1[1][0] = vc_objects[i]->trans[1][0];
                              R1[1][1] = vc_objects[i]->trans[1][1];
                              R1[1][2] = vc_objects[i]->trans[1][2];
                              R1[2][0] = vc_objects[i]->trans[2][0];
                              R1[2][1] = vc_objects[i]->trans[2][1];
                              R1[2][2] = vc_objects[i]->trans[2][2];

                              T1[0] = vc_objects[i]->trans[0][3];
                              T1[1] = vc_objects[i]->trans[1][3];
                              T1[2] = vc_objects[i]->trans[2][3];

                              //set up the rotation and translation matrices for
                              //the second object.
                              R2[0][0] = vc_objects[curr_ovrlp->id]->trans[0][0];
                              R2[0][1] = vc_objects[curr_ovrlp->id]->trans[0][1];
                              R2[0][2] = vc_objects[curr_ovrlp->id]->trans[0][2];
                              R2[1][0] = vc_objects[curr_ovrlp->id]->trans[1][0];
                              R2[1][1] = vc_objects[curr_ovrlp->id]->trans[1][1];
                              R2[1][2] = vc_objects[curr_ovrlp->id]->trans[1][2];
                              R2[2][0] = vc_objects[curr_ovrlp->id]->trans[2][0];
                              R2[2][1] = vc_objects[curr_ovrlp->id]->trans[2][1];
                              R2[2][2] = vc_objects[curr_ovrlp->id]->trans[2][2];

                              T2[0] = vc_objects[curr_ovrlp->id]->trans[0][3];
                              T2[1] = vc_objects[curr_ovrlp->id]->trans[1][3];
                              T2[2] = vc_objects[curr_ovrlp->id]->trans[2][3];

                              if (flag == VC_FIRST_CONTACT)
                              {
                                //call the RAPID collision detection routine.
                                RAPID_Collide(R1, T1, vc_objects[i]->b,
                                              R2, T2, vc_objects[curr_ovrlp->id]->b,
                                              RAPID_FIRST_CONTACT);

                                //if there is a collision, then add the pair to the
                                //collision report database.
                                if (RAPID_num_contacts != 0)
                                  report_data.AddPair(i, vc_objects[curr_ovrlp->id]->id);
                              }
                              else
                              {
                                //call the RAPID collision detection routine.
                                RAPID_Collide(R1, T1, vc_objects[i]->b,
                                              R2, T2, vc_objects[curr_ovrlp->id]->b,
                                              RAPID_ALL_CONTACTS);

                                //if there is a collision, then add the pair to the
                                //collision report database.
                                if (RAPID_num_contacts != 0)
                                  report_data.AddPair(i, vc_objects[curr_ovrlp->id]->id,
                                                      RAPID_contact, RAPID_num_contacts);
                              }

                            }
                        }
                      else if (curr_disabled->id == curr_ovrlp->id)
                        {
                          curr_disabled = curr_disabled->next;
                        }
                    }
                  else
                    {
                      if (vc_objects[curr_ovrlp->id]->activation_state == 1)
                        {
                          //again, we need to call the RAPID collision detection routine.

                          double R1[3][3], T1[3], R2[3][3], T2[3];

                          //set up the rotation and translation matrices as
                          //required by RAPID for the first object.
                          R1[0][0] = vc_objects[i]->trans[0][0];
                          R1[0][1] = vc_objects[i]->trans[0][1];
                          R1[0][2] = vc_objects[i]->trans[0][2];
                          R1[1][0] = vc_objects[i]->trans[1][0];
                          R1[1][1] = vc_objects[i]->trans[1][1];
                          R1[1][2] = vc_objects[i]->trans[1][2];
                          R1[2][0] = vc_objects[i]->trans[2][0];
                          R1[2][1] = vc_objects[i]->trans[2][1];
                          R1[2][2] = vc_objects[i]->trans[2][2];

                          T1[0] = vc_objects[i]->trans[0][3];
                          T1[1] = vc_objects[i]->trans[1][3];
                          T1[2] = vc_objects[i]->trans[2][3];

                          //set up the rotation and translation matrices for
                          //the second object.
                          R2[0][0] = vc_objects[curr_ovrlp->id]->trans[0][0];
                          R2[0][1] = vc_objects[curr_ovrlp->id]->trans[0][1];
                          R2[0][2] = vc_objects[curr_ovrlp->id]->trans[0][2];
                          R2[1][0] = vc_objects[curr_ovrlp->id]->trans[1][0];
                          R2[1][1] = vc_objects[curr_ovrlp->id]->trans[1][1];
                          R2[1][2] = vc_objects[curr_ovrlp->id]->trans[1][2];
                          R2[2][0] = vc_objects[curr_ovrlp->id]->trans[2][0];
                          R2[2][1] = vc_objects[curr_ovrlp->id]->trans[2][1];
                          R2[2][2] = vc_objects[curr_ovrlp->id]->trans[2][2];

                          T2[0] = vc_objects[curr_ovrlp->id]->trans[0][3];
                          T2[1] = vc_objects[curr_ovrlp->id]->trans[1][3];
                          T2[2] = vc_objects[curr_ovrlp->id]->trans[2][3];

                          if (flag == VC_FIRST_CONTACT)
                          {
                            //call the RAPID collision detection routine.
                            RAPID_Collide(R1, T1, vc_objects[i]->b,
                                          R2, T2, vc_objects[curr_ovrlp->id]->b,
                                          RAPID_FIRST_CONTACT);

                            //if there is a collision, then add the pair to the
                            //collision report database.
                            if (RAPID_num_contacts != 0)
                              report_data.AddPair(i, vc_objects[curr_ovrlp->id]->id);
                          }
                          else
                          {
                            //call the RAPID collision detection routine.
                            RAPID_Collide(R1, T1, vc_objects[i]->b,
                                          R2, T2, vc_objects[curr_ovrlp->id]->b,
                                          RAPID_ALL_CONTACTS);

                            //if there is a collision, then add the pair to the
                            //collision report database.
                            if (RAPID_num_contacts != 0)
                              report_data.AddPair(i, vc_objects[curr_ovrlp->id]->id,
                                                  RAPID_contact, RAPID_num_contacts);
                          }

                        }
                    }
                  curr_ovrlp = curr_ovrlp->next;
                }
            }
        }
    }

  Report( report );

  return VC_OK;
}



void VCInternal::Report(VCReport *report)
{
    report->num_obj_pairs = 0;
    delete [] report->obj_pair;
    report->obj_pair = NULL;

    int count = 0;
    int i;

    // count the total number of object pairs.
    for ( i = 0; i < report_data.size; i++ )
    {
	Elem *current;
	for ( current = report_data.arr[i]; current != NULL; current = current->next )
	    count++;
    }

    if ( count > 0 )
	report->obj_pair = new VCObjPair[count];
    else
	report->obj_pair = NULL;

    report->num_obj_pairs = count;

    // copy report
    count = 0;

    for ( i = 0; i < report_data.size; i++ )
    {
	Elem *current;
	for ( current = report_data.arr[i]; current != NULL; current = current->next )
	{
	    report->obj_pair[count].id1 = i;
	    report->obj_pair[count].id2 = current->id;
	    report->obj_pair[count].num_tri_pairs = current->num_contacts;
	    report->obj_pair[count].tri_pair = current->contact;
	    current->num_contacts = 0;
	    current->contact = NULL;
	    count++;
	}
    }
}
