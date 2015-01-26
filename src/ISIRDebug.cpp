/**
 * \file ISIRDebug.cpp
 * \author Joseph Salini
 *
 * \brief Implementation to get debugging information.
 */

#include "orcisir/ISIRDebug.h"
#include <fstream>
#include <ios>

//std::ofstream debugfile("/tmp/sot-core-traces.txt",
//			  std::ios::trunc&std::ios::out);
//IsirDebugTrace isirDEBUGFLOW(debugfile);
IsirDebugTrace isirDEBUGFLOW(std::cout);

IsirDebugTrace::IsirDebugTrace(std::ostream& os)
	: outputbuffer(os){
}
