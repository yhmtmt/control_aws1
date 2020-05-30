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


extern  const char * str_aws1_ctrl_src[ControlSource_NONE];

class f_control_aws1: public f_base
{
 protected:
  // ch_aws1* are removed soon. ch_ctrl_data the replacment.
  ch_ctrl_data * m_ch_ctrl_out;         // (ui<-autopilot<-control)
  ch_ctrl_data * m_ch_ctrl_in;          // (ui->autopilot->control)
  unsigned char buf[256];
  unsigned int buf_len;
  flatbuffers::FlatBufferBuilder builder;

  unsigned char rud_normal, eng_normal, rud, eng;
  bool bconfig; // configuration completed flag
  unsigned char eng_max, eng_nuf, eng_nut, eng_nub, eng_min,
    rud_max, rud_nut, rud_min;

  
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
  vector<int> m_eng_smpl;
  vector<int> m_rud_sta_smpl;

  void lpf();
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
