/**
 * @author: Nicola Dessi'
 */
#pragma once
#include <ariadne.h>
#include "controllers.hh"
#include "tanks.hh"
#include "valves.hh"
namespace Ariadne
{
HybridIOAutomaton getSystem(
	double T_val = 1.0, 
	double H1_val = 1.0, 
	double H2_val = 2.0, 
	double H3_val = 5.0, 
	double delta_val = 0.1,
	double alpha12_val = 0.02,
	double alpha23_val = 0.02,
	double alpha32_val = 0.02,
	double alpha21_val = 0.02,
	double alpha13_val = 0.02,
	double alpha31_val = 0.02)
{
	RealParameter T("T", T_val);
	RealParameter H1("H1", H1_val);
	RealParameter H2("H2", H2_val);
	RealParameter H3("H3", H3_val);
	RealParameter delta("delta", delta_val);
	RealParameter alpha12("alpha12", alpha12_val);
	RealParameter alpha23("alpha23", alpha23_val);
	RealParameter alpha32("alpha32", alpha32_val);
	RealParameter alpha21("alpha21", alpha21_val);
	RealParameter alpha13("alpha13", alpha13_val);
	RealParameter alpha31("alpha31", alpha31_val);

	DiscreteLocation S0("S0");
	DiscreteLocation flow("flow");
	
	RealVariable z1("z1");
	RealVariable z2("z2");
	RealVariable z3("z3");
	RealVariable a12("a12");
	RealVariable a13("a13");
	RealVariable a23("a23");
	RealVariable a21("a21");
	RealVariable a31("a31");
	RealVariable a32("a32");

	DiscreteEvent a("a");
	DiscreteEvent b("b");
	DiscreteEvent c("c");
	DiscreteEvent d("d");
	DiscreteEvent e("e");
	DiscreteEvent f("f");
	DiscreteEvent g("g");
	DiscreteEvent h("h");
	DiscreteEvent i("i");
	DiscreteEvent l("l");
	DiscreteEvent m("m");

	//Valve a12
	DiscreteEvent e_idle12("e_idle12");

	DiscreteLocation opening12("opening12");
	DiscreteLocation closing12("closing12");
	DiscreteLocation idle12("idle12");
	//Valve a13
	DiscreteEvent e_idle13("e_idle13");

	DiscreteLocation opening13("opening13");
	DiscreteLocation closing13("closing13");
	DiscreteLocation idle13("idle13");	
	//Valve a23
	DiscreteEvent e_idle23("e_idle23");

	DiscreteLocation opening23("opening23");
	DiscreteLocation closing23("closing23");
	DiscreteLocation idle23("idle23");	
	//Valve a21
	DiscreteEvent e_idle21("e_idle21");

	DiscreteLocation opening21("opening21");
	DiscreteLocation closing21("closing21");
	DiscreteLocation idle21("idle21");	
	//Valve a31
	DiscreteEvent e_idle31("e_idle31");

	DiscreteLocation opening31("opening31");
	DiscreteLocation closing31("closing31");
	DiscreteLocation idle31("idle31");
	//Valve a32
	DiscreteEvent e_idle32("e_idle32");

	DiscreteLocation opening32("opening32");
	DiscreteLocation closing32("closing32");
	DiscreteLocation idle32("idle32");



	HybridIOAutomaton valve13 = Valve13::getSystem("valve13",a13,T,a,i,b,l,e_idle13,idle13,opening13,closing13);
	HybridIOAutomaton valve12 = Valve12::getSystem("valve12",a12,T,c,h,a,d,i,e_idle12,idle12,opening12,closing12);
	HybridIOAutomaton valve23 = Valve23::getSystem("valve23",a23,T,b,g,l,c,h,m,e_idle23,idle23,opening23,closing23);
	HybridIOAutomaton valve21 = Valve21::getSystem("valve21",a21,T,f,g,e_idle21,idle21,opening21,closing21);
	HybridIOAutomaton valve31 = Valve31::getSystem("valve31",a31,T,e,f,e_idle31,idle31,opening31,closing31);
	HybridIOAutomaton valve32 = Valve32::getSystem("valve32",a32,T,d,e,e_idle32,idle32,opening32,closing32);
	
	HybridIOAutomaton controller = Controller::getSystem(z1,z2,z3,a,b,c,d,e,f,g,h,i,l,m,H1,H2,H3,S0);

	HybridIOAutomaton tank = Flow::getSystem(z1,z2,z3,a12,a13,a23,a21,a32,a31,alpha12,alpha23,alpha32,alpha21,alpha13,alpha31,flow);

	HybridIOAutomaton s1 = compose("s1",tank,valve12,flow,opening12);
	HybridIOAutomaton s2 = compose("s2",s1,valve13,DiscreteLocation("flow,opening12"),idle13);
	HybridIOAutomaton s3 = compose("s3",s2,valve23,DiscreteLocation("flow,opening12,idle13"),idle23);
	HybridIOAutomaton s4 = compose("s4",s3,valve21,DiscreteLocation("flow,opening12,idle13,idle23"),idle21);
	HybridIOAutomaton s5 = compose("s5",s4,valve31,DiscreteLocation("flow,opening12,idle13,idle23,idle21"),idle31);
	HybridIOAutomaton s6 = compose("s6",s5,valve32,DiscreteLocation("flow,opening12,idle13,idle23,idle21,idle31"),idle32);
	HybridIOAutomaton system = compose("hanoi",s6,controller,DiscreteLocation("flow,opening12,idle13,idle23,idle21,idle31,idle32"),S0);

	return system;

}
}