//emacs -*-Mode: C++;-*-
/*!
  @file iface_ssv.hpp

  obstacles from vision system

  Copyright (C)  Institute of Applied Mechanics, TU-Muenchen
   All rights reserved.
   

*/
#ifndef __IFACE_SSV_HPP__
#define __IFACE_SSV_HPP__
#include <stdint.h>
#include <begin_pack.h>

#include <list>
#include <vector>

#include <xstdio.h>
#include <xdebug.h>

#include <iface_vision_msg.hpp>
#include <iface_stepseq.hpp>


namespace am2b_iface
{

   
  //! artifical environment: format to add a new obstacle to environment 
  // consist of ssv approximation or surface approximation and 
  // wrl representation (for dynamic simulation)
  class ArtEnvData
  {
  public:
    ArtEnvData():
      url_wrl("default"),
      url_ssv("default"),
      has_wrl_file(false),
      has_ssv_file(false),
      has_surface_file(false)
    {
      for(uint16_t ii = 0; ii<3 ; ++ii)
        {
          r_trans[ii] = 0;
          kardan_angle[ii] = 0;
        }
    };
    virtual ~ArtEnvData(){};
    //! simulated environment
    std::string url_wrl;
    //! control internal representation of world
    //! ssv description
    std::string url_ssv;
    //! surface description
    std::string url_surface;

    //! has obstacle description a corresponding wrl/ssv/surface file?
    bool has_wrl_file;
    bool has_ssv_file;
    bool has_surface_file;

    //! translation of object
    float r_trans[3];
    //! rotation of object described using kardan angle
    float kardan_angle[3];
  };

  

  /***********************************************************************
   *    PARAMETERS FOR SSV 
   ***********************************************************************/

  //! variable if the whole segment or only a ssv obj of the segment should be deleted
  //! this is for the old lepp and should be removed if lepp3 works
  enum{REMOVE_SSV_SEG=0,REMOVE_SSV_OBJ=1,REMOVE_SSV_NONE=-1}; 
  //! variable for corresponding surface - no surface ->NO_SURFACE otherwise surface id
  enum{NO_SURFACE=-1}; 
  //iface wird auch zum loggen in ssv map verwendet!
  struct struct_data_ssv
  {
    //! id of segment
    int32_t id;
    //! id of ssv object (not used : 0)
    int32_t id_ssv;
    //! together with delete event: should only ssv obj be deleted or whole segment?
    int32_t remove_ssv_obj;
    //! type -> 0:PSS, 1:LSS, 2:TSS
    int32_t type;
    //! Which is the coresseponding surface? 
    int32_t surface;
    //! radius
    float radius;
    //! corresponding stamp
    int32_t stamp_gen;
    //! vectors
    std::vector< std::vector<float> > p;
    
    //! missing: function to calculate event data from iface and vice versa


    struct_data_ssv()
    {    
      reset();
    };

    void reset()
    {
      id = -1;
      id_ssv = 0;
      remove_ssv_obj = 0;
      type = 0;
      surface = -1;
      radius = 0;
      stamp_gen = -1;
      // vectors
      p.reserve(3);
      std::vector<float> tmp(3,0);
      for(uint64_t ii = 0; ii<3; ++ii)
        {
	  p.push_back(tmp);
        }
    };

    void print() const
    {
      pdbg("SSV Data:\n" 
	   "type / id / id_ssv / remove_ssv_obj / surface / stamp: %d / %d / %d / %d / %d / %d\n",
	   type,id,id_ssv,remove_ssv_obj,surface,stamp_gen);
	
      pdbg("radius %e\n",radius);

      pdbg("p0 [%e %e %e]\n p1 [%e %e %e]\n p2 [%e %e %e]\n",
	   p[0][0],p[0][1],p[0][2],
	   p[1][0],p[1][1],p[1][2],
	   p[2][0],p[2][1],p[2][2]);
    }

    
    struct_data_ssv& operator=(const struct_data_stepseq_ssv_log& rhs_)
    {
      //! stamp of ssv
      stamp_gen = rhs_.stamp_gen;

      //! id of segment
      id = rhs_.id;
      //! id of ssv object (not used : 0)
      id_ssv = rhs_.id_ssv;
      //! together with delete event: should only ssv obj be deleted or whole segment?
      remove_ssv_obj = rhs_.remove_ssv_obj;
      //! type -> 0:PSS, 1:LSS, 2:TSS
      type = rhs_.type;
      //! Which is the coresseponding surface? 
      surface = rhs_.surface;
      //! radius
      radius = rhs_.radius;
      //! vectors
      p[0][0] = rhs_.p0[0];
      p[0][1] = rhs_.p0[1];
      p[0][2] = rhs_.p0[2];
      
      p[1][0] = rhs_.p1[0];
      p[1][1] = rhs_.p1[1];
      p[1][2] = rhs_.p1[2];

      p[2][0] = rhs_.p2[0];
      p[2][1] = rhs_.p2[1];
      p[2][2] = rhs_.p2[2];

      return *this;
    }

    void get_data_log(struct_data_stepseq_ssv_log& log_data_) const
    { 
      // do not get stamp data
      log_data_.type = this->type;
      log_data_.id = this->id;
      log_data_.id_ssv = this->id_ssv;
      log_data_.radius = this->radius;
      log_data_.remove_ssv_obj = this->remove_ssv_obj;
      log_data_.surface = this->surface;
      // vectors
      for(uint64_t ii = 0; ii<3;++ii)
        {
          log_data_.p0[ii] = this->p[0][ii];
          log_data_.p1[ii] = this->p[1][ii];
          log_data_.p2[ii] = this->p[2][ii];
        }
    }

    struct_data_ssv& operator=(const ObstacleMessage& msg)
    {
      
      this->id = msg.model_id;
      this->id_ssv = msg.part_id;
      if(msg.action == REMOVE_SSV_ONLY_PART)
	this->remove_ssv_obj =   msg.action;
      else
	this->remove_ssv_obj = 0;
      this->type = msg.type;
      this->radius =   msg.radius;
      this->surface =  msg.surface;
      // data_ssv(6,7,8) = vec p0,
      this->p[0][0] =  msg.coeffs[0];
      this->p[0][1] =   msg.coeffs[1];
      this->p[0][2] =   msg.coeffs[2];
      // data_ssv(9,10,11)) = vec p1,
      this->p[1][0]  =   msg.coeffs[3];
      this->p[1][1] =   msg.coeffs[4];
      this->p[1][2] =   msg.coeffs[5];
      // data_ssv(12,115_,14)) = vec p2
      this->p[2][0] =   msg.coeffs[6];
      this->p[2][1] =  msg.coeffs[7];
      this->p[2][2] =  msg.coeffs[8];
      return *this;

    }

    struct_data_ssv& operator=(const struct_data_ssv& in_)
    {
      
      this->stamp_gen = in_.stamp_gen;
      this->id = in_.id;
      this->id_ssv = in_.id_ssv;
      this->remove_ssv_obj =   in_.remove_ssv_obj;
      this->type = in_.type;
      this->radius =   in_.radius;
      this->surface =  in_.surface;
      this->stamp_gen = in_.stamp_gen;
      // data_ssv(6,7,8) = vec p0,
      this->p[0][0] =  in_.p[0][0];
      this->p[0][1] =   in_.p[0][1];
      this->p[0][2] =   in_.p[0][2];
      // data_ssv(9,10,11)) = vec p1,
      this->p[1][0]  =   in_.p[1][0];
      this->p[1][1] =   in_.p[1][1];
      this->p[1][2] =   in_.p[1][2];
      // data_ssv(12,115_,14)) = vec p2
      this->p[2][0] =   in_.p[2][0];
      this->p[2][1] =  in_.p[2][1];
      this->p[2][2] =  in_.p[2][2];
      return *this;

    }
    


  };



/***********************************************************************
   *    PARAMETERS FOR SURFACES 
   ***********************************************************************/

  //surface discription
  struct struct_data_surface
  {
    
    struct_data_surface()
    {      
      stamp_gen = -1;
      id = -1;
      // vectors
      n.reserve(3);
      for (uint16_t ii = 0; ii<3; ++ii)
        {
          n.push_back(0);
        }
      p.reserve(8);
      // std::vector<float> tmp(3);
      // for (uint16_t ii = 0; ii<3; ++ii)
      //   {
      //     tmp[ii] = 0;
      //   }      
      // for (uint16_t ii = 0; ii<8; ++ii)
      //   {
      //     p.push_back(tmp);
      //   }      
    }

    struct_data_surface& operator=(const SurfaceMessage& msg)
    {
      this->id =   msg.id;
      // normal of surface
      for(uint64_t ii = 0; ii<3;ii++)
	this->n[ii] = msg.normal[ii];
      // corner points
      int counter = 0;
      this->p.clear();
      std::vector<float> tmp(3);
      //msg.vertices.size() is alway assumed to be 24 / 8 corners
      for (size_t ii = 0; ii<8;++ii)
	{
	  for(size_t jj = 0; jj<3; ++jj)
	    {
	      tmp[jj] = msg.vertices[counter];  
	      ++counter;
	    }
	  this->p.push_back(tmp);
	}
      return *this;

    }
    // its not defined how many corners a surface should have, 
    // but in the default case it has 8 corner points
    void create_default_surface()
    {      
      stamp_gen = -1;
      id = -1;      
      n.reserve(3);
      for (uint16_t ii = 0; ii<3; ++ii)
        {
          n.push_back(0);
        }
      std::vector<float> tmp(3);
      tmp.reserve(3);
      for (uint16_t ii = 0; ii<3; ++ii)
        {
          tmp[ii] = 0;
        }      
      for (uint16_t ii = 0; ii<8; ++ii)
        {
          p.push_back(tmp);
        } 
    }

    struct_data_surface& operator=(const struct_data_stepseq_ssv_log& rhs_)
    {
      this->stamp_gen =   rhs_.stamp_gen;
      this->id =   rhs_.surface_id;
      // normal of surface
      for(uint64_t ii = 0; ii<3;ii++)
	this->n[ii] = rhs_.surface_n[ii];
      // corner points
      int counter = 0;
      this->p.clear();
      std::vector<float> tmp(3);
      //msg.vertices.size() is alway assumed to be 24 / 8 corners

      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p0[jj];  
          ++counter;
        }
      this->p.push_back(tmp);

      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p1[jj];  
          ++counter;
        }
      this->p.push_back(tmp);

      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p2[jj];  
          ++counter;
        }
      this->p.push_back(tmp);
      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p3[jj];  
          ++counter;
        }
      this->p.push_back(tmp);
      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p4[jj];  
          ++counter;
        }
      this->p.push_back(tmp);
      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p5[jj];  
          ++counter;
        }
      this->p.push_back(tmp);
      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p6[jj];  
          ++counter;
        }
      this->p.push_back(tmp);
      for(size_t jj = 0; jj<3; ++jj)
        {
          tmp[jj] = rhs_.surface_p7[jj];  
          ++counter;
        }
      this->p.push_back(tmp);


      return *this;
    }

    void get_data_log(struct_data_stepseq_ssv_log& log_data_) const
    {
      // do not get stamp data
      if(this->p.size() != 8)
        {
          perr_ffl("Till now: assuming that surfaces have 8 corner!\n");
          return;
        }
      log_data_.surface_id = this->id;
      for(uint64_t ii = 0; ii<3;++ii)
        {
          log_data_.surface_n[ii] = this->n[ii];
          log_data_.surface_p0[ii] = this->p[0][ii];
          log_data_.surface_p1[ii] = this->p[1][ii];
          log_data_.surface_p2[ii] = this->p[2][ii];
          log_data_.surface_p3[ii] = this->p[3][ii];
          log_data_.surface_p4[ii] = this->p[4][ii];
          log_data_.surface_p5[ii] = this->p[5][ii];
          log_data_.surface_p6[ii] = this->p[6][ii];
          log_data_.surface_p7[ii] = this->p[7][ii];
        }
    }


    void print() const
    {
      pdbg("Surface data:\n");
      pdbg("id %d\n",this->id);
      pdbg("normal [");
      // normal of surface
      for(uint64_t ii = 0; ii<n.size();ii++)
	pdbg("%e ",this->n[ii]);
      pdbg("]\n");
      pdbg("corner [");
      //msg.vertices.size() is alway assumed to be 24 / 8 corners
      for (size_t ii = 0; ii<p.size();++ii)
	{
	  for(size_t jj = 0; jj<p[ii].size(); ++jj)
	    {
	      pdbg("%e ",this->p[ii][jj]);
	    }
	  pdbg("\n");
	}
      pdbg("]\n");
    }

    //! corresponding stamp (time of construction)
    int32_t stamp_gen;    
    //! id of surface
    int32_t id;
    //! normal
    std::vector<float> n;
    //! vectors
    std::vector< std::vector<float> > p;   
  };



}
#include <end_pack.h>
#endif//__IFACE_SSV_HPP__
