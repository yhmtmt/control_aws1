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
  m_ch_ctrl_out(nullptr), m_ch_ctrl_in(nullptr),
  bconfig(false),
  rud_normal(127), eng_normal(127), rud(127), eng(127),
  m_adclpf(false), m_sz_adclpf(5), m_cur_adcsmpl(0), m_sigma_adclpf(3.0),
  builder(256)
{
  strcpy(m_dev, "/dev/zgpio1");
  m_flog_name[0] = 0;
 
  register_fpar("ch_ctrl_out", (ch_base**)&m_ch_ctrl_out,
		typeid(ch_ctrl_data).name(), "Control data out (to autopilot)");
  register_fpar("ch_ctrl_in", (ch_base**)&m_ch_ctrl_in,
		typeid(ch_ctrl_data).name(), "Control data in (from autopilot)");
  register_fpar("device", m_dev, 1023, "AWS1's control gpio device path");
  register_fpar("flog", m_flog_name, 1023, "Control log file.");
  register_fpar("sim", &m_sim, "Simulation mode.");
  register_fpar("verb", &m_verb, "For debug.");
  // LPF related parameters
  register_fpar("adclpf", &m_adclpf, "LPF is applied for the ADC inputs.");
  register_fpar("sz_adclpf", &m_sz_adclpf, "Window size of the ADC-LPF.");
  register_fpar("type_adclpf", (int*) &m_type_adclpf, ADCLPF_NONE, m_str_adclpf_type, "Type of ADCLPF.");
  register_fpar("sigma_adclpf", &m_sigma_adclpf, "Standard deviation of the gaussian kernel of the ADC-LPF (This can only be used in the case of the filter type is gauss)");

  // aws's control parameters
  register_fpar("awsrud", &rud_normal, "Control value of AWS1's rudder.");
  register_fpar("awseng", &eng_normal, "Control value of AWS1's main engine.");

  // Each control points of the main engine output.
  register_fpar("eng_max", &eng_max, "Maximum control value for AWS1's main engine.");
  register_fpar("eng_nuf", &eng_nuf, "Nutral to Forward control value for AWS1's main engine.");
  register_fpar("eng_nut", &eng_nut, "Nutral control value for AWS1's main engine.");
  register_fpar("eng_nub", &eng_nub, "Nutral to Backward control value for AWS1's main engine.");
  register_fpar("eng_min", &eng_min, "Minimum control value for AWS1's main engine.");

  // Each controll points of the rudder output.
  register_fpar("rud_max", &rud_max, "Maximum control value for AWS1's rudder.");
  register_fpar("rud_nut", &rud_nut, "Nutral control value for AWS1's rudder.");
  register_fpar("rud_min", &rud_min, "Minimum control value for AWS1's rudder.");

  register_fpar("eng", &eng, "Output value for main engine.");
  register_fpar("rud", &rud, "Output value for rudder.");
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

  bconfig = false;
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
  rud = map_oval(rud_normal,
		 0xff, 0x7f, 0x00, 
		 rud_max, rud_nut, rud_min);
  eng = map_oval(eng_normal, 
		 0xff, 0x7f + 0x19, 0x7f, 0x7f - 0x19, 0x00,
		 eng_max, eng_nuf, eng_nut, 
		 eng_nub, eng_min);  
  ((unsigned char *) &val)[2] = rud;
  ((unsigned char *) &val)[3] = eng;
    
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

  if(m_flog.is_open()){
    m_flog << m_cur_time << " ";
    m_flog << (int) rud_normal << " " << (int) eng_normal << " " ;
    m_flog << (int) rud << " " << (int) eng << " " ;
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
  double rud = 0., eng = 0.;
  
  for(int i = 0, j = m_cur_adcsmpl; i < m_sz_adclpf; i++, j = (j + 1) % m_sz_adclpf ){
    rud += m_rud_smpl[j] * m_kern_adclpf[i];
    eng += m_eng_smpl[j] * m_kern_adclpf[i];
  }
  
  m_cur_adcsmpl = (m_cur_adcsmpl > 0 ? m_cur_adcsmpl - 1 : m_sz_adclpf - 1);
}

void f_control_aws1::set_stat()
{
  if(m_ch_ctrl_out && !bconfig){
    builder.Clear();
    auto payload = Control::CreateConfig(builder, eng_max, eng_nuf, eng_nut, eng_nub, eng_min, rud_max, rud_nut, rud_min);
    auto data = CreateData(builder, get_time(),
			   Control::Payload_Config, payload.Union());
    builder.Finish(data);
    m_ch_ctrl_out->push(builder.GetBufferPointer(), builder.GetSize());
    
  }  
}

void f_control_aws1::get_inst()
{  
  while(1){
    if(m_ch_ctrl_in){
      m_ch_ctrl_in->pop(buf, buf_len);
      auto data = Control::GetData(buf);
      switch(data->payload_type()){
      case Control::Payload_Engine:
	eng_normal = (unsigned char) (data->payload_as_Engine()->value());
	if(m_ch_ctrl_out) m_ch_ctrl_out->push(buf, buf_len);
	break;
      case Control::Payload_Rudder:
	rud_normal = (unsigned char) (data->payload_as_Engine()->value());
	if(m_ch_ctrl_out) m_ch_ctrl_out->push(buf, buf_len);	
	break;
      case Control::Payload_Config:
	{
	auto config = data->payload_as_Config();
	if(eng_max == config->engine_max() && 
	   eng_min == config->engine_min() &&
	   eng_nut == config->engine_nutral() && 
	   eng_nuf == config->engine_forward() &&
	   eng_nub == config->engine_backward() &&
	   rud_max == config->rudder_max() &&
	   rud_nut == config->rudder_mid() &&
	   rud_min == config->rudder_min())
	  bconfig = true;
	}
	break;
      }
    }
  }
}
