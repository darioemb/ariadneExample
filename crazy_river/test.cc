#include<ariadne.h>
#include"crazy_river.hh"

#include<iostream>


int main(int argc, char** argv)
{
	HybridIOAutomaton crazy_river=getSystem();

	std::cout<<"Automaton: "<<crazy_river<<"\n";

}
