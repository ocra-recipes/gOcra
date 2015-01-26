/**
 * \file ISIRModel.h
 * \author Joseph Salini
 *
 * \brief Define method to load a orc::Model directly from a shared library .
 */

#ifndef __ORCISIRMODEL_H__
#define __ORCISIRMODEL_H__

#include <iostream>
#include <stdlib.h> // to get relative path of file

#include <dlfcn.h> // To load shared libraries

#include "orc/control/Model.h"


namespace orcisir
{

orc::Model* getModelFromSharedLibrary(const std::string& libPath, const std::string& createFunctionName, const std::string& robotName)
{
    char  actualpath[4096];
    realpath(libPath.c_str(), actualpath);

    void* SL_Handle;
    SL_Handle = dlopen(actualpath, RTLD_LAZY);
    if(SL_Handle == NULL){
       throw std::runtime_error(std::string("[orcisir::getModelFromSharedLibrary]: Cannot load shared library: '")+std::string(actualpath)+"'" );
    }
    orc::Model* (*create_SL_Model)(const std::string&);
    *(void **)(&create_SL_Model) = dlsym(SL_Handle, createFunctionName.c_str());

    return (*create_SL_Model)(robotName);

}


}

#endif
