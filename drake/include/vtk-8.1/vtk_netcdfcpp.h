/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtk_netcdfcxx.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef vtk_netcdfcpp_h
#define vtk_netcdfcpp_h

/* Use the netcdfcpp library configured for VTK.  */
#define VTK_USE_SYSTEM_NETCDFCPP
#ifdef VTK_USE_SYSTEM_NETCDFCPP
# include <netcdfcpp.h>
#else
# include <vtknetcdfcpp/netcdfcpp.h>
#endif

#endif
