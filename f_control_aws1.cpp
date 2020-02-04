// Copyright(c) 2016-2019 Yohei Matsumoto, All right reserved. 

// f_control_aws1.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_control_aws1.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_control_aws1.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include  "zgpio.h"
#include "f_control_aws1.hpp"
DEFINE_FILTER(f_control_aws1)
  
const char * f_control_aws1:: m_str_adclpf_type[ADCLPF_NONE] = {
  "avg", "gauss"
};

f_control_aws1::f_control_aws1(const char * name): 
  f_base(name),  m_fd(-1), m_sim(false), m_verb(false),
  m_ch_ctrl_ui(NULL), m_ch_ctrl_ap1(NULL), m_ch_ctrl_ap2(NULL),  m_ch_ctrl_stat(NULL), 
  m_adclpf(false), m_sz_adclpf(5), m_cur_adcsmpl(0), m_sigma_adclpf(3.0)
{
  strcpy(m_dev, "/dev/zgpio1");
  m_flog_name[0] = 0;

  register_fpar("ch_ctrl_ui", (ch_base**)&m_ch_ctrl_ui, typeid(ch_aws1_ctrl_inst).name(), "Channel of the AWS1's control inputs from UI.");
  register_fpar("ch_ctrl_ap1", (ch_base**)&m_ch_ctrl_ap1, typeid(ch_aws1_ctrl_inst).name(), "Channel of the AWS1's control inputs from AutoPilot1.");
  register_fpar("ch_ctrl_ap2", (ch_base**)&m_ch_ctrl_ap2, typeid(ch_aws1_ctrl_inst).name(), "Channel of the AWS1's control inputs from AutoPilot2.");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ch_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Channel of the AWS1 control outputs.");
  register_fpar("device", m_dev, 1023, "AWS1's control gpio device path");
  register_fpar("flog", m_flog_name, 1023, "Control log file.");
  register_fpar("sim", &m_sim, "Simulation mode.");
  register_fpar("verb", &m_verb, "For debug.");
  register_fpar("acs", (int*) &m_stat.ctrl_src, ACS_NONE, str_aws1_ctrl_src,  "AWS control source.");
  // LPF related parameters
  register_fpar("adclpf", &m_adclpf, "LPF is applied for the ADC inputs.");
  register_fpar("sz_adclpf", &m_sz_adclpf, "Window size of the ADC-LPF.");
  register_fpar("type_adclpf", (int*) &m_type_adclpf, ADCLPF_NONE, m_str_adclpf_type, "Type of ADCLPF.");
  register_fpar("sigma_adclpf", &m_sigma_adclpf, "Standard deviation of the gaussian kernel of the ADC-LPF (This can only be used in the case of the filter type is gauss)");

  // aws's control parameters
  register_fpar("awsrud", &m_stat.rud_aws, "Control value of AWS1's rudder.");
  register_fpar("awsmeng", &m_stat.meng_aws, "Control value of AWS1's main engine.");

  // Remote controllers control points of the main engine. 
  register_fpar("meng_max_rmc", &m_stat.meng_max_rmc, "Maximum control control value of AWS1's main engine controller.");
  register_fpar("meng_nuf_rmc", &m_stat.meng_nuf_rmc, "Nutral to Forward control value of AWS1's main engine controller.");
  register_fpar("meng_nut_rmc", &m_stat.meng_nut_rmc, "Nutral control value of AWS1's main engine controller.");
  register_fpar("meng_nub_rmc", &m_stat.meng_nub_rmc, "Nutral to Backward control value of AWS1's main engine controller.");
  register_fpar("meng_min_rmc", &m_stat.meng_min_rmc, "Minimum control value of AWS1's main engine controller.");

  // Each control points of the main engine output.
  register_fpar("meng_max", &m_stat.meng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("meng_nuf", &m_stat.meng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("meng_nut", &m_stat.meng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("meng_nub", &m_stat.meng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("meng_min", &m_stat.meng_min, "Minimum control value for AWS1's main engine.");

  // Remote controllers control points of the sub engine.

  // Remote controller's control points of the rudder.
  register_fpar("rud_max_rmc", &m_stat.rud_max_rmc, "Maximum control value of AWS1's rudder controller.");
  register_fpar("rud_nut_rmc", &m_stat.rud_nut_rmc, "Nutral control value of AWS1's rudder controller.");
  register_fpar("rud_min_rmc", &m_stat.rud_min_rmc, "Minimum control value of AWS1's rudder controller.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &m_stat.rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &m_stat.rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &m_stat.rud_min, "Minimum control value for AWS1's rudder.");

  register_fpar("meng", &m_stat.meng, "Output value for main engine.");
  register_fpar("rud", &m_stat.rud, "Output value for rudder.");
}

f_control_aws1::~f_control_aws1()
{
}

bool f_control_aws1::init_run()
{
  if(!m_sim){
    m_fd = open(m_dev, O_RDWR);
    if(m_fd == -1){
      cerr << "Error in f_control_aws1::init_run, opening device " << m_dev << "." << endl; 
      cerr << "    Message: " << strerror(errno) << endl;
      return false;
    }
  }

  if(m_flog_name[0] != 0){
    m_flog.open(m_flog_name);
    if(!m_flog.is_open()){
      cerr << m_name <<  " failed to open log file." << endl;
      return false;
    }
  }

  return true;
}

void f_control_aws1::destroy_run()
{
  if(m_fd != -1)
    close(m_fd);
}

void f_control_aws1::get_gpio()
{
}

// In this method, aws's control values are assumed to be 0-255.
// Then the values are mapped to actual control values.
// 
// Rudder
// 0-127: port (min to nutral)
// 127-255: starboard (nutral to max)
//
// Engine 
// 0-102: Astern (min to nutral backword)
// 102-127: Deadslow Astern (nutral backword to nutral)
// 127-132: Deadslow Ahead (nutral to nutral forward)
// 132-255: Ahead (nutral forward to max)

void f_control_aws1::set_gpio()
{
  unsigned int val;

  switch(m_stat.ctrl_src){
  case ACS_UI:
  case ACS_AP1:
  case ACS_AP2:
  case ACS_FSET:
  case ACS_NONE:
    m_stat.rud = map_oval(m_stat.rud_aws, 
		     0xff, 0x7f, 0x00, 
		     m_stat.rud_max, m_stat.rud_nut, m_stat.rud_min);
    m_stat.meng = map_oval(m_stat.meng_aws, 
		      0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		      m_stat.meng_max, m_stat.meng_nuf, m_stat.meng_nut, 
		      m_stat.meng_nub, m_stat.meng_min);  
    break;
  }
  
  ((unsigned char *) &val)[2] = m_stat.rud;
  ((unsigned char *) &val)[3] = m_stat.meng;
    
  if(!m_sim){
    ioctl(m_fd, ZGPIO_IOCSET2, &val);
  }
}

bool f_control_aws1::proc()
{
  get_inst();

  get_gpio();
  if(m_adclpf)
    lpf();

  set_gpio();

  if(m_verb){
    cout << "Control Values." << endl;
    cout << "    aws rud " << (int) m_stat.rud_aws << " meng " << (int) m_stat.meng_aws << endl;
    cout << "    out rud " << (int) m_stat.rud << " meng " << (int) m_stat.meng << endl;

  }
  if(m_flog.is_open()){
    m_flog << m_cur_time << " ";
    m_flog << (int) m_stat.rud_aws << " " << (int) m_stat.meng_aws << " " ;
    m_flog << (int) m_stat.rud << " " << (int) m_stat.meng << " " ;
  }

  set_stat();
  return true;
}

void f_control_aws1::lpf()
{
  if(m_sz_adclpf != m_kern_adclpf.size()){ // initialize filter
    if(m_verb)
    // allocating memory
    m_kern_adclpf.resize(m_sz_adclpf);

    // building filter kernel.
    switch(m_type_adclpf){
    case ADCLPF_GAUSS:
      {
	double c = (double)(m_sz_adclpf - 1) / 2.0;
	double sum = 0.;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] = (float) gauss(c, m_sigma_adclpf, (double) i);
	  sum += m_kern_adclpf[i];
	}

	// normalize the filter.
	sum = 1.0 / sum;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] *= sum;
	}
      }
      break;
    case ADCLPF_AVG:
      {
	double val = 1.0 / (double) m_sz_adclpf;
	for(int i = 0; i < m_sz_adclpf; i++){
	  m_kern_adclpf[i] = (float) val;
	}
      }
      break;
    default:
      break;
    }

    m_cur_adcsmpl = 0;
    if(m_verb)
      cout << " done." << endl;
  }

  // kernel convolution
  double rud = 0., meng = 0.;
  
  for(int i = 0, j = m_cur_adcsmpl; i < m_sz_adclpf; i++, j = (j + 1) % m_sz_adclpf ){
    rud += m_rud_smpl[j] * m_kern_adclpf[i];
    meng += m_meng_smpl[j] * m_kern_adclpf[i];
  }
  
  m_cur_adcsmpl = (m_cur_adcsmpl > 0 ? m_cur_adcsmpl - 1 : m_sz_adclpf - 1);
}

void f_control_aws1::set_stat()
{
  m_stat.tcur = m_cur_time;
  m_ch_ctrl_stat->set(m_stat);
}

void f_control_aws1::get_inst()
{

  s_aws1_ctrl_inst inst;
  if(m_ch_ctrl_ui){
    m_ch_ctrl_ui->get(inst);
  }

  m_stat.tcur = inst.tcur;
  m_stat.ctrl_src = inst.ctrl_src;

  switch(m_stat.ctrl_src){
  case ACS_UI:
    m_stat.rud_aws = inst.rud_aws;
    m_stat.meng_aws = inst.meng_aws;
    break;
  case ACS_AP1:
    if(m_ch_ctrl_ap1){
      m_ch_ctrl_ap1->get(inst);
      m_stat.rud_aws = inst.rud_aws;
      m_stat.meng_aws = inst.meng_aws;
    }else{
      cerr << "In " << m_name << ", ";
      cerr << "No autopilot channel 1 is connected" << endl;
    }
    break;
  case ACS_AP2:
    if(m_ch_ctrl_ap2){
      m_ch_ctrl_ap2->get(inst);
      m_stat.rud_aws = inst.rud_aws;
      m_stat.meng_aws = inst.meng_aws;
    }else{
      cerr << "In " << m_name << ", ";
      cerr << "No autopilot channel 2 is connected" << endl;
    }
    break;
  }
}
