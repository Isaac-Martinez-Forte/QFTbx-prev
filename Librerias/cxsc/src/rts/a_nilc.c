/*
**  CXSC is a C++ library for eXtended Scientific Computing (V 2.5.4)
**
**  Copyright (C) 1990-2000 Institut fuer Angewandte Mathematik,
**                          Universitaet Karlsruhe, Germany
**            (C) 2000-2014 Wiss. Rechnen/Softwaretechnologie
**                          Universitaet Wuppertal, Germany   
**
**  This library is free software; you can redistribute it and/or
**  modify it under the terms of the GNU Library General Public
**  License as published by the Free Software Foundation; either
**  version 2 of the License, or (at your option) any later version.
**
**  This library is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
**  Library General Public License for more details.
**
**  You should have received a copy of the GNU Library General Public
**  License along with this library; if not, write to the Free
**  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/* CVS $Id: a_nilc.c,v 1.21 2014/01/30 17:24:02 cxsc Exp $ */

/****************************************************************/
/*                                                              */
/*      Filename        : a_nilc.c                              */
/*                                                              */
/*      Entries         : a_VOID a_nilc(a)                      */
/*                        a_VOID a;                             */
/*                                                              */
/*      Arguments       : a = pointer                           */
/*                                                              */
/*      Description     : Test for NULL pointer                 */
/*                                                              */
/*      Function value  : Input pointer.                        */
/*                                                              */
/****************************************************************/

#ifndef ALL_IN_ONE
#ifdef AIX
#include "/u/p88c/runtime/o_defs.h"
#else
#include "o_defs.h"
#endif
#define local
#endif

#ifdef LINT_ARGS
local a_VOID a_nilc(a_VOID a)
#else
local a_VOID a_nilc(a)

a_VOID a;
#endif
        {
        E_TPUSH("a_nilc")

        if (a==NULL)
           e_trap(NO_ERROR+E_EXIT,2,E_TMSG,24);

        E_TPOPP("a_nilc")
        return(a);
        }





