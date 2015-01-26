/**
 * \file ISIRDebug.h
 * \author Sovannara Hak, Joseph Salini
 *
 * \brief Debug trace
 *
 * Inspired by François Bleibel, Olivier Stasse
 * define ISIR_DEBUG to activate tracing
 * define trace level ISIR_DEBUG_MODE
 * use isirDEBUG(level) or isirTOTALDEBUG(level) like ostream
 */

#ifndef ISIRDEBUG_H
# define ISIRDEBUG_H

//#define ISIR_DEBUG
#define ISIR_DEBUG_MODE 0


# include <cstdio>
# include <iostream>
# include <fstream>
# include <sstream>

# ifndef ISIR_DEBUG_MODE
#  define ISIR_DEBUG_MODE 0
# endif

class IsirDebugTrace{
	public :
		IsirDebugTrace(std::ostream& os);

		std::ostream& outputbuffer;

};
extern IsirDebugTrace isirDEBUGFLOW;

# ifdef ISIR_DEBUG
#  define isirPREDEBUG 					\
	__FILE__ << ": " << __FUNCTION__ 	\
	<< "(#" << __LINE__ << ") :"

#  define isirTOTALDEBUG(level)													\
	if ((level>ISIR_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer << isirPREDEBUG

#  define isirDEBUG(level)													\
	if ((level>ISIR_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer
# else // ISIR_DEBUG
#  define isirTOTALDEBUG(level) if( 1 ) ; else std::cout
#  define isirDEBUG(level) if( 1 ) ; else std::cout

#endif // ISIR_DEBUG

#endif //ifndef ISIRDEBUG_H
