// Copyright(c) 2016-2019 Yohei Matsumoto, All right reserved. 

// f_control_aws1.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_control_aws1.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_control_aws1.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifndef F_CONTROL_AWS1_HPP
#define F_CONTROL_AWS1_HPP

#include "ch_vector.hpp"
#include "ch_aws1_ctrl.hpp"

#include "filter_base.hpp"


extern  const char * str_aws1_ctrl_src[ACS_NONE];

class f_control_aws1: public f_base
{
 protected:
  ch_aws1_ctrl_inst * m_ch_ctrl_ui, * m_ch_ctrl_ap1, * m_ch_ctrl_ap2;
  ch_aws1_ctrl_stat * m_ch_ctrl_stat;   // channel for control parameters
  char m_dev[1024];         // device path, e.g. "/dev/zgpio0"
  char m_flog_name[1024];
  ofstream m_flog;
  int m_fd;                 // file descriptor for zgpio
  
  // for simulation
  bool m_sim;
  float m_rud_sta_sim;

  bool m_verb;
 
  /// LPF related parameters
  bool m_adclpf;           // Enabling ADC's low pass filter.
  int m_sz_adclpf;         // Window size of the low pass filter.
  int m_cur_adcsmpl;       // Current position in the adc sample buffer
  float m_sigma_adclpf;    // Standard deviation of the gaussian kernel.

  enum e_adclpf_type{
    ADCLPF_AVG, ADCLPF_GAUSS, ADCLPF_NONE
  } m_type_adclpf;
  static const char * m_str_adclpf_type[ADCLPF_NONE];
  
  vector<float> m_kern_adclpf;
  vector<int> m_rud_smpl;
  vector<int> m_meng_smpl;
  vector<int> m_seng_smpl;
  vector<int> m_rud_sta_smpl;

  void lpf();

  s_aws1_ctrl_stat m_stat;

  void get_gpio();
  void set_gpio();

  void set_stat();
  void get_inst();

public:
  f_control_aws1(const char * name);
  
  virtual ~f_control_aws1();
  
  virtual bool init_run();
  
  virtual void destroy_run();
  
  virtual bool proc();
};

#endif
