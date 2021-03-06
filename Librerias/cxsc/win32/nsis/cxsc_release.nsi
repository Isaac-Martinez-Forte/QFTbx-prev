; Script generated by the HM NIS Edit Script Wizard.

; HM NIS Edit Wizard helper defines
!define PRODUCT_NAME "C-XSC"
!define PRODUCT_VERSION "Version 2.5.4"
!define PRODUCT_PUBLISHER "WRSWT - Bergische Universitšt Wuppertal"
!define PRODUCT_WEB_SITE "http://www.xsc.de"
!define PRODUCT_DIR_REGKEY "Software\Microsoft\Windows\CurrentVersion\App Paths\make_test.exe"
!define PRODUCT_UNINST_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PRODUCT_NAME}"
!define PRODUCT_UNINST_ROOT_KEY "HKLM"

; MUI 1.67 compatible ------
!include "MUI.nsh"
!include "AddToPath.nsh"

; MUI Settings
!define MUI_ABORTWARNING
!define MUI_ICON "${NSISDIR}\Contrib\Graphics\Icons\modern-install.ico"
!define MUI_UNICON "${NSISDIR}\Contrib\Graphics\Icons\modern-uninstall.ico"

; Welcome page
!insertmacro MUI_PAGE_WELCOME
; License page
!define MUI_LICENSEPAGE_RADIOBUTTONS
!insertmacro MUI_PAGE_LICENSE "..\..\docu\COPYING"
; Directory page
!insertmacro MUI_PAGE_DIRECTORY
; Instfiles page
!insertmacro MUI_PAGE_INSTFILES
; Finish page
!define MUI_FINISHPAGE_RUN "$INSTDIR\examples\test\make_test.exe"
!insertmacro MUI_PAGE_FINISH

; Uninstaller pages
!insertmacro MUI_UNPAGE_INSTFILES

; Language files
!insertmacro MUI_LANGUAGE "English"

; MUI end ------

Name "${PRODUCT_NAME} ${PRODUCT_VERSION}"
OutFile "Setup.exe"
InstallDir "c:\cxsc"
InstallDirRegKey HKLM "${PRODUCT_DIR_REGKEY}" ""
ShowInstDetails show
ShowUnInstDetails show

Section "include" SEC01
  SetOutPath "$INSTDIR\include"
  SetOverwrite ifnewer
  File "..\..\CToolbox\Modules\cilss.hpp"
  File "..\..\CToolbox\Modules\clss.hpp"
  File "..\..\CToolbox\Modules\ilss.hpp"
  File "..\..\CToolbox\Modules\lss.hpp"
  File "..\..\CToolbox\Modules\fastlss.hpp"
  File "..\..\CToolbox\Modules\fastlss.inl"
  File "..\..\CToolbox\Modules\matinv_aprx.hpp"
  File "..\..\CToolbox\Modules\set_ari.hpp"
  File "..\..\CToolbox\Modules\rpoly.hpp"
  File "..\..\CToolbox\Modules\rpeval.hpp"
  File "..\..\CToolbox\Modules\rev_simp.hpp"
  File "..\..\CToolbox\Modules\r_util.hpp"
  File "..\..\CToolbox\Modules\nlinsys.hpp"
  File "..\..\CToolbox\Modules\nlfzero.hpp"
  File "..\..\CToolbox\Modules\mvi_util.hpp"
  File "..\..\CToolbox\Modules\mv_util.hpp"
  File "..\..\CToolbox\Modules\matinv.hpp"
  File "..\..\CToolbox\Modules\lst_ari.hpp"
  File "..\..\CToolbox\Modules\lst1_ari.hpp"
  File "..\..\CToolbox\Modules\lop_ari.hpp"
  File "..\..\CToolbox\Modules\lop.hpp"
  File "..\..\CToolbox\Modules\linsys.hpp"
  File "..\..\CToolbox\Modules\i_util.hpp"
  File "..\..\CToolbox\Modules\hess_ari.hpp"
  File "..\..\CToolbox\Modules\grad_ari.hpp"
  File "..\..\CToolbox\Modules\gop.hpp"
  File "..\..\CToolbox\Modules\gop1.hpp"
  File "..\..\CToolbox\Modules\expreval.hpp"
  File "..\..\CToolbox\Modules\ddf_ari.hpp"
  File "..\..\CToolbox\Modules\cpzero.hpp"
  File "..\..\CToolbox\Modules\cpoly.hpp"
  File "..\..\CToolbox\Modules\cipoly.hpp"
  File "..\..\CToolbox\Modules\ci_util.hpp"
  File "..\..\CToolbox\Modules\stacksz.hpp"
  File "..\..\CToolbox\Modules\*.inl"
  File "..\cxscconf.h"
  File "..\..\src\dotk.inl"
  File "..\..\src\cdotk.inl"
  File "..\..\src\idotk.inl"
  File "..\..\src\cidotk.inl"
  File "..\..\src\sparsedot.hpp"
  File "..\..\src\sparsecdot.hpp"
  File "..\..\src\sparseidot.hpp"
  File "..\..\src\sparsecidot.hpp"
  File "..\..\src\sparsedot.inl"
  File "..\..\src\sparseidot.inl"
  File "..\..\src\sparsecdot.inl"
  File "..\..\src\sparsematrix.hpp"
  File "..\..\src\sparsematrix.inl"
  File "..\..\src\sparsevector.hpp"
  File "..\..\src\sparsevector.inl"
  File "..\..\src\matrix_friend_declarations.inl"
  File "..\..\src\vector_friend_declarations.inl"
  File "..\..\src\srmatrix.hpp"
  File "..\..\src\simatrix.hpp"
  File "..\..\src\scmatrix.hpp"
  File "..\..\src\scimatrix.hpp"
  File "..\..\src\srvector.hpp"
  File "..\..\src\sivector.hpp"
  File "..\..\src\scivector.hpp"
  File "..\..\src\scvector.hpp"
  File "..\..\src\lx_real.hpp"
  File "..\..\src\lx_real.inl"
  File "..\..\src\lx_interval.hpp"
  File "..\..\src\lx_interval.inl"
  File "..\..\src\lx_complex.hpp"
  File "..\..\src\lx_complex.inl"
  File "..\..\src\lx_cinterval.hpp"
  File "..\..\src\lx_cinterval.inl"
  File "..\..\src\lx_ivector.hpp"
  File "..\..\src\lx_ivector.inl"
  File "..\..\src\lx_civector.hpp"
  File "..\..\src\lx_civector.inl"
  File "..\..\src\vector.hpp"
  File "..\..\src\testvect.hpp"
  File "..\..\src\testsklr.hpp"
  File "..\..\src\testmatr.hpp"
  File "..\..\src\testintv.hpp"
  File "..\..\src\testdot.hpp"
  File "..\..\src\testcomp.hpp"
  File "..\..\src\testclss.hpp"
  File "..\..\src\test.hpp"
  File "..\..\src\rvector.hpp"
  File "..\..\src\RtsTyp.h"
  File "..\..\src\rtsrmath.h"
  File "..\..\src\RtsFunc.h"
  File "..\..\src\rts_real.hpp"
  File "..\..\src\rmatrix.hpp"
  File "..\..\src\rmath.hpp"
  File "..\..\src\real.hpp"
  File "..\..\src\matrix.hpp"
  File "..\..\src\lrvecrmat.hpp"
  File "..\..\src\lrvecivec.hpp"
  File "..\..\src\lrvecimat.hpp"
  File "..\..\src\lrmatimat.hpp"
  File "..\..\src\livecrmat.hpp"
  File "..\..\src\liveclrmat.hpp"
  File "..\..\src\livecimat.hpp"
  File "..\..\src\l_rvector.hpp"
  File "..\..\src\l_rmatrix.hpp"
  File "..\..\src\l_rmath.hpp"
  File "..\..\src\l_real.hpp"
  File "..\..\src\l_ivector.hpp"
  File "..\..\src\l_interval.hpp"
  File "..\..\src\l_imatrix.hpp"
  File "..\..\src\l_imath.hpp"
  File "..\..\src\l_complex.hpp"
  File "..\..\src\ivector.hpp"
  File "..\..\src\ivecrmat.hpp"
  File "..\..\src\iveclrmat.hpp"
  File "..\..\src\iveccvec.hpp"
  File "..\..\src\iveccmat.hpp"
  File "..\..\src\ioflags.hpp"
  File "..\..\src\intvector.hpp"
  File "..\..\src\intmatrix.hpp"
  File "..\..\src\interval.hpp"
  File "..\..\src\imatrix.hpp"
  File "..\..\src\imath.hpp"
  File "..\..\src\imath.inl"
  File "..\..\src\idot.hpp"
  File "..\..\src\except.hpp"
  File "..\..\src\dot_defs.hpp"
  File "..\..\src\dot.hpp"
  File "..\..\src\cxscvect.hpp"
  File "..\..\src\cxscmatr.hpp"
  File "..\..\src\cvector.hpp"
  File "..\..\src\cvecrmat.hpp"
  File "..\..\src\cvecimat.hpp"
  File "..\..\src\complex.hpp"
  File "..\..\src\compiler.h"
  File "..\..\src\cmatrix.hpp"
  File "..\..\src\cmatimat.hpp"
  File "..\..\src\civector.hpp"
  File "..\..\src\civecrmat.hpp"
  File "..\..\src\civecimat.hpp"
  File "..\..\src\civeccmat.hpp"
  File "..\..\src\cinterval.hpp"
  File "..\..\src\cimatrix.hpp"
  File "..\..\src\cimath.hpp"
  File "..\..\src\cidot.hpp"
  File "..\..\src\cdot.hpp"
  File "..\..\src\xscclass.hpp"
  File "..\..\src\fi_lib\fi_lib.hpp"
  File "..\..\src\fi_lib\fi_lib_consts.hpp"
  File "..\..\src\rts\y_fcth.h"
  File "..\..\src\rts\t_name.h"
  File "..\..\src\rts\t_mach.h"
  File "..\..\src\rts\t_ieee.h"
  File "..\..\src\rts\t_fcth.h"
  File "..\..\src\rts\t_exc_.h"
  File "..\..\src\rts\t_drea.h"
  File "..\..\src\rts\t_defs.h"
  File "..\..\src\rts\t_ddev.h"
  File "..\..\src\rts\t_cond.h"
  File "..\..\src\rts\t_cnst.h"
  File "..\..\src\rts\subbody.h"
  File "..\..\src\rts\s_fcth.h"
  File "..\..\src\rts\r_fcth.h"
  File "..\..\src\rts\r_defs.h"
  File "..\..\src\rts\p88rts.h"
  File "..\..\src\rts\o_type.h"
  File "..\..\src\rts\o_syst.h"
  File "..\..\src\rts\o_spec.h"
  File "..\..\src\rts\o_slct.h"
  File "..\..\src\rts\o_revs.h"
  File "..\..\src\rts\o_name.h"
  File "..\..\src\rts\o_msg1.h"
  File "..\..\src\rts\o_fcth.h"
  File "..\..\src\rts\o_defs.h"
  File "..\..\src\rts\mulbody.h"
  File "..\..\src\rts\l_fcth.h"
  File "..\..\src\rts\l_defs.h"
  File "..\..\src\rts\f_fcth.h"
  File "..\..\src\rts\e_fcth.h"
  File "..\..\src\rts\e_defs.h"
  File "..\..\src\rts\divtrap.h"
  File "..\..\src\rts\divbody.h"
  File "..\..\src\rts\d_fcth.h"
  File "..\..\src\rts\d_defs.h"
  File "..\..\src\rts\body.h"
  File "..\..\src\rts\b_lpi_.h"
  File "..\..\src\rts\b_lari.h"
  File "..\..\src\rts\b_fcth.h"
  File "..\..\src\rts\b_defs.h"
  File "..\..\src\rts\b_64bt.h"
  File "..\..\src\rts\addbody.h"
  File "..\..\src\rts\a_fcth.h"
  File "..\..\src\rts\a_defs.h"
SectionEnd

Section "lib" SEC02
  SetOutPath "$INSTDIR\lib"
  File "..\..\Release\cxsc.lib"
SectionEnd

Section "examples" SEC03
  SetOutPath "$INSTDIR\examples"
  File "..\CToolbox\cpz_ex\Release\cpz_ex.exe"
  File "..\CToolbox\ddf_ex\Release\ddf_ex.exe"
  File "..\CToolbox\gop1_ex\Release\gop1_ex.exe"
  File "..\CToolbox\gop_ex\Release\gop_ex.exe"
  File "..\CToolbox\hess_ex\Release\hess_ex.exe"
  File "..\CToolbox\jac_ex\Release\jac_ex.exe"
  File "..\CToolbox\lop_ex\Release\lop_ex.exe"
  File "..\CToolbox\lss_ex\Release\lss_ex.exe"
  File "..\CToolbox\fastlss_ex\Release\fastlss_ex.exe"
  File "..\CToolbox\nlfz_ex\Release\nlfz_ex.exe"
  File "..\CToolbox\nlss_ex\Release\nlss_ex.exe"
  File "..\CToolbox\rpe_ex\Release\rpe_ex.exe"
  File "..\CToolbox\xev_ex1\Release\xev_ex1.exe"
  File "..\CToolbox\xev_ex2\Release\xev_ex2.exe"
  File "..\CToolbox\xev_ex3\Release\xev_ex3.exe"
SectionEnd

Section "vgl" SEC04
  SetOutPath "$INSTDIR\examples\test"
  File "..\..\CToolbox\test\xev_ex2.vgl"
  File "..\..\CToolbox\test\xev_ex1.vgl"
  File "..\..\CToolbox\test\tester.vgl"
  File "..\..\CToolbox\test\rpe_ex.vgl2"
  File "..\..\CToolbox\test\rpe_ex.vgl1"
  File "..\..\CToolbox\test\nlss_ex.vgl2"
  File "..\..\CToolbox\test\nlss_ex.vgl1"
  File "..\..\CToolbox\test\nlfz_ex.vgl"
  File "..\..\CToolbox\test\lss_ex.vgl2"
  File "..\..\CToolbox\test\lss_ex.vgl1"
  File "..\..\CToolbox\test\lop_ex.vgl2"
  File "..\..\CToolbox\test\lop_ex.vgl1"
  File "..\..\CToolbox\test\jac_ex.vgl"
  File "..\..\CToolbox\test\hess_ex.vgl"
  File "..\..\CToolbox\test\gop_ex.vgl"
  File "..\..\CToolbox\test\gop1_ex.vgl"
  File "..\..\CToolbox\test\cpz_ex.vgl2"
  File "..\..\CToolbox\test\cpz_ex.vgl1"
  File "..\..\CToolbox\test\xev_ex3.vgl"
  File "..\..\CToolbox\test\ergebnis.dat"
SectionEnd

Section "in" SEC05
  File "..\..\CToolbox\Programs\xev_ex2.in"
  File "..\..\CToolbox\Programs\xev_ex1.in"
  File "..\..\CToolbox\Programs\rpe_ex.in2"
  File "..\..\CToolbox\Programs\rpe_ex.in1"
  File "..\..\CToolbox\Programs\nlss_ex.in2"
  File "..\..\CToolbox\Programs\nlss_ex.in1"
  File "..\..\CToolbox\Programs\nlfz_ex.in"
  File "..\..\CToolbox\Programs\lss_ex.in2"
  File "..\..\CToolbox\Programs\lss_ex.in1"
  File "..\..\CToolbox\Programs\lop_ex.in2"
  File "..\..\CToolbox\Programs\lop_ex.in1"
  File "..\..\CToolbox\Programs\jac_ex.in"
  File "..\..\CToolbox\Programs\hess_ex.in"
  File "..\..\CToolbox\Programs\gop_ex.in"
  File "..\..\CToolbox\Programs\gop1_ex.in"
  File "..\..\CToolbox\Programs\cpz_ex.in2"
  File "..\..\CToolbox\Programs\cpz_ex.in1"
  File "..\..\CToolbox\Programs\xev_ex3.in"
SectionEnd

Section "inline" SEC06
  SetOutPath "$INSTDIR\include"
  File "..\..\src\rvector.inl"
  File "..\..\src\rmatrix.inl"
  File "..\..\src\rmath.inl"
  File "..\..\src\real.inl"
  File "..\..\src\matrix.inl"
  File "..\..\src\lrvecrmat.inl"
  File "..\..\src\lrvecivec.inl"
  File "..\..\src\lrvecimat.inl"
  File "..\..\src\lrmatimat.inl"
  File "..\..\src\livecrmat.inl"
  File "..\..\src\liveclrmat.inl"
  File "..\..\src\livecimat.inl"
  File "..\..\src\l_rvector.inl"
  File "..\..\src\l_rmatrix.inl"
  File "..\..\src\l_rmath.inl"
  File "..\..\src\l_ivector.inl"
  File "..\..\src\l_interval.inl"
  File "..\..\src\l_imatrix.inl"
  File "..\..\src\l_complex.inl"
  File "..\..\src\ivector.inl"
  File "..\..\src\ivecrmat.inl"
  File "..\..\src\iveclrmat.inl"
  File "..\..\src\iveccvec.inl"
  File "..\..\src\iveccmat.inl"
  File "..\..\src\intvector.inl"
  File "..\..\src\intmatrix.inl"
  File "..\..\src\interval.inl"
  File "..\..\src\imatrix.inl"
  File "..\..\src\idot.inl"
  File "..\..\src\cvector.inl"
  File "..\..\src\cvecrmat.inl"
  File "..\..\src\cvecimat.inl"
  File "..\..\src\complex.inl"
  File "..\..\src\cmatrix.inl"
  File "..\..\src\cmatimat.inl"
  File "..\..\src\civector.inl"
  File "..\..\src\civecrmat.inl"
  File "..\..\src\civecimat.inl"
  File "..\..\src\civeccmat.inl"
  File "..\..\src\cinterval.inl"
  File "..\..\src\cimatrix.inl"
  File "..\..\src\cidot.inl"
  File "..\..\src\cdot.inl"
  File "..\..\src\vector.inl"
SectionEnd

Section "test" SEC07
  SetOutPath "$INSTDIR\examples\test"
  File "..\CToolbox\make_test\make_test.txt"
  File "..\CToolbox\make_test\Release\make_test.exe"
  File "..\CToolbox\tester\Release\tester.exe"
  File "..\CToolbox\sumup\Release\sumup.exe"
SectionEnd

Section "myexample" SEC08
  SetOutPath "$INSTDIR\examples\project"
  File "..\myexample\myexample.sln"
  SetOutPath "$INSTDIR\examples\project\myexample"
  File "..\myexample\myexample\myexample.vcxproj"
  File "..\myexample\myexample\myexample.h"
  File "..\myexample\myexample\myexample.cpp"
SectionEnd

Section -AdditionalIcons
  SetOutPath $INSTDIR
  WriteIniStr "$INSTDIR\${PRODUCT_NAME}.url" "InternetShortcut" "URL" "${PRODUCT_WEB_SITE}"
  CreateDirectory "$SMPROGRAMS\cxsc"
  CreateShortCut "$SMPROGRAMS\cxsc\Website.lnk" "$INSTDIR\${PRODUCT_NAME}.url"
  CreateShortCut "$SMPROGRAMS\cxsc\Uninstall.lnk" "$INSTDIR\uninst.exe"
SectionEnd

Section -Post
  WriteUninstaller "$INSTDIR\uninst.exe"
  WriteRegStr HKLM "${PRODUCT_DIR_REGKEY}" "" "$INSTDIR\examples\cpz_ex.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayName" "$(^Name)"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "UninstallString" "$INSTDIR\uninst.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayIcon" "$INSTDIR\examples\cpz_ex.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "DisplayVersion" "${PRODUCT_VERSION}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "URLInfoAbout" "${PRODUCT_WEB_SITE}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}" "Publisher" "${PRODUCT_PUBLISHER}"
SectionEnd


Function un.onUninstSuccess
  HideWindow
  MessageBox MB_ICONINFORMATION|MB_OK "$(^Name) wurde erfolgreich deinstalliert."
FunctionEnd

Function un.onInit
  MessageBox MB_ICONQUESTION|MB_YESNO|MB_DEFBUTTON2 "MŲchten Sie $(^Name) und alle seinen Komponenten deinstallieren?" IDYES +2
  Abort
FunctionEnd

Section Uninstall
  Delete "$INSTDIR\${PRODUCT_NAME}.url"
  Delete "$INSTDIR\uninst.exe"
  Delete "$INSTDIR\examples\test\make_test.exe"
  Delete "$INSTDIR\examples\test\make_test.txt"
  Delete "$INSTDIR\include\vector.inl"
  Delete "$INSTDIR\include\cdot.inl"
  Delete "$INSTDIR\include\cidot.inl"
  Delete "$INSTDIR\include\cimatrix.inl"
  Delete "$INSTDIR\include\cinterval.inl"
  Delete "$INSTDIR\include\civeccmat.inl"
  Delete "$INSTDIR\include\civecimat.inl"
  Delete "$INSTDIR\include\civecrmat.inl"
  Delete "$INSTDIR\include\civector.inl"
  Delete "$INSTDIR\include\cmatimat.inl"
  Delete "$INSTDIR\include\cmatrix.inl"
  Delete "$INSTDIR\include\complex.inl"
  Delete "$INSTDIR\include\cvecimat.inl"
  Delete "$INSTDIR\include\cvecrmat.inl"
  Delete "$INSTDIR\include\cvector.inl"
  Delete "$INSTDIR\include\idot.inl"
  Delete "$INSTDIR\include\imatrix.inl"
  Delete "$INSTDIR\include\interval.inl"
  Delete "$INSTDIR\include\intmatrix.inl"
  Delete "$INSTDIR\include\intvector.inl"
  Delete "$INSTDIR\include\iveccmat.inl"
  Delete "$INSTDIR\include\iveccvec.inl"
  Delete "$INSTDIR\include\iveclrmat.inl"
  Delete "$INSTDIR\include\ivecrmat.inl"
  Delete "$INSTDIR\include\ivector.inl"
  Delete "$INSTDIR\include\l_complex.inl"
  Delete "$INSTDIR\include\l_imatrix.inl"
  Delete "$INSTDIR\include\l_interval.inl"
  Delete "$INSTDIR\include\l_ivector.inl"
  Delete "$INSTDIR\include\l_rmath.inl"
  Delete "$INSTDIR\include\l_rmatrix.inl"
  Delete "$INSTDIR\include\l_rvector.inl"
  Delete "$INSTDIR\include\livecimat.inl"
  Delete "$INSTDIR\include\liveclrmat.inl"
  Delete "$INSTDIR\include\livecrmat.inl"
  Delete "$INSTDIR\include\lrmatimat.inl"
  Delete "$INSTDIR\include\lrvecimat.inl"
  Delete "$INSTDIR\include\lrvecivec.inl"
  Delete "$INSTDIR\include\lrvecrmat.inl"
  Delete "$INSTDIR\include\matrix.inl"
  Delete "$INSTDIR\include\real.inl"
  Delete "$INSTDIR\include\rmath.inl"
  Delete "$INSTDIR\include\rmatrix.inl"
  Delete "$INSTDIR\include\rvector.inl"
  Delete "$INSTDIR\include\*.inl"
  Delete "$INSTDIR\examples\test\xev_ex3.in"
  Delete "$INSTDIR\examples\test\cpz_ex.in1"
  Delete "$INSTDIR\examples\test\cpz_ex.in2"
  Delete "$INSTDIR\examples\test\gop1_ex.in"
  Delete "$INSTDIR\examples\test\gop_ex.in"
  Delete "$INSTDIR\examples\test\hess_ex.in"
  Delete "$INSTDIR\examples\test\jac_ex.in"
  Delete "$INSTDIR\examples\test\lop_ex.in1"
  Delete "$INSTDIR\examples\test\lop_ex.in2"
  Delete "$INSTDIR\examples\test\lss_ex.in1"
  Delete "$INSTDIR\examples\test\lss_ex.in2"
  Delete "$INSTDIR\examples\test\nlfz_ex.in"
  Delete "$INSTDIR\examples\test\nlss_ex.in1"
  Delete "$INSTDIR\examples\test\nlss_ex.in2"
  Delete "$INSTDIR\examples\test\rpe_ex.in1"
  Delete "$INSTDIR\examples\test\rpe_ex.in2"
  Delete "$INSTDIR\examples\test\xev_ex1.in"
  Delete "$INSTDIR\examples\test\xev_ex2.in"
  Delete "$INSTDIR\examples\test\xev_ex3.vgl"
  Delete "$INSTDIR\examples\test\cpz_ex.vgl1"
  Delete "$INSTDIR\examples\test\cpz_ex.vgl2"
  Delete "$INSTDIR\examples\test\gop1_ex.vgl"
  Delete "$INSTDIR\examples\test\gop_ex.vgl"
  Delete "$INSTDIR\examples\test\hess_ex.vgl"
  Delete "$INSTDIR\examples\test\jac_ex.vgl"
  Delete "$INSTDIR\examples\test\lop_ex.vgl1"
  Delete "$INSTDIR\examples\test\lop_ex.vgl2"
  Delete "$INSTDIR\examples\test\lss_ex.vgl1"
  Delete "$INSTDIR\examples\test\lss_ex.vgl2"
  Delete "$INSTDIR\examples\test\nlfz_ex.vgl"
  Delete "$INSTDIR\examples\test\nlss_ex.vgl1"
  Delete "$INSTDIR\examples\test\nlss_ex.vgl2"
  Delete "$INSTDIR\examples\test\rpe_ex.vgl1"
  Delete "$INSTDIR\examples\test\rpe_ex.vgl2"
  Delete "$INSTDIR\examples\test\tester.vgl"
  Delete "$INSTDIR\examples\test\xev_ex1.vgl"
  Delete "$INSTDIR\examples\test\xev_ex2.vgl"
  Delete "$INSTDIR\examples\xev_ex3.exe"
  Delete "$INSTDIR\examples\xev_ex2.exe"
  Delete "$INSTDIR\examples\xev_ex1.exe"
  Delete "$INSTDIR\examples\test\tester.exe"
  Delete "$INSTDIR\examples\test\sumup.exe"
  Delete "$INSTDIR\examples\rpe_ex.exe"
  Delete "$INSTDIR\examples\nlss_ex.exe"
  Delete "$INSTDIR\examples\nlfz_ex.exe"
  Delete "$INSTDIR\examples\lss_ex.exe"
  Delete "$INSTDIR\examples\fastlss_ex.exe"
  Delete "$INSTDIR\examples\lop_ex.exe"
  Delete "$INSTDIR\examples\jac_ex.exe"
  Delete "$INSTDIR\examples\hess_ex.exe"
  Delete "$INSTDIR\examples\gop_ex.exe"
  Delete "$INSTDIR\examples\gop1_ex.exe"
  Delete "$INSTDIR\examples\ddf_ex.exe"
  Delete "$INSTDIR\examples\cpz_ex.exe"
  Delete "$INSTDIR\lib\cxsc.lib"
  Delete "$INSTDIR\include\a_defs.h"
  Delete "$INSTDIR\include\a_fcth.h"
  Delete "$INSTDIR\include\addbody.h"
  Delete "$INSTDIR\include\b_64bt.h"
  Delete "$INSTDIR\include\b_defs.h"
  Delete "$INSTDIR\include\b_fcth.h"
  Delete "$INSTDIR\include\b_lari.h"
  Delete "$INSTDIR\include\b_lpi_.h"
  Delete "$INSTDIR\include\body.h"
  Delete "$INSTDIR\include\d_defs.h"
  Delete "$INSTDIR\include\d_fcth.h"
  Delete "$INSTDIR\include\divbody.h"
  Delete "$INSTDIR\include\divtrap.h"
  Delete "$INSTDIR\include\e_defs.h"
  Delete "$INSTDIR\include\e_fcth.h"
  Delete "$INSTDIR\include\f_fcth.h"
  Delete "$INSTDIR\include\l_defs.h"
  Delete "$INSTDIR\include\l_fcth.h"
  Delete "$INSTDIR\include\mulbody.h"
  Delete "$INSTDIR\include\o_defs.h"
  Delete "$INSTDIR\include\o_fcth.h"
  Delete "$INSTDIR\include\o_msg1.h"
  Delete "$INSTDIR\include\o_name.h"
  Delete "$INSTDIR\include\o_revs.h"
  Delete "$INSTDIR\include\o_slct.h"
  Delete "$INSTDIR\include\o_spec.h"
  Delete "$INSTDIR\include\o_syst.h"
  Delete "$INSTDIR\include\o_type.h"
  Delete "$INSTDIR\include\p88rts.h"
  Delete "$INSTDIR\include\r_defs.h"
  Delete "$INSTDIR\include\r_fcth.h"
  Delete "$INSTDIR\include\s_fcth.h"
  Delete "$INSTDIR\include\subbody.h"
  Delete "$INSTDIR\include\t_cnst.h"
  Delete "$INSTDIR\include\t_cond.h"
  Delete "$INSTDIR\include\t_ddev.h"
  Delete "$INSTDIR\include\t_defs.h"
  Delete "$INSTDIR\include\t_drea.h"
  Delete "$INSTDIR\include\t_exc_.h"
  Delete "$INSTDIR\include\t_fcth.h"
  Delete "$INSTDIR\include\t_ieee.h"
  Delete "$INSTDIR\include\t_mach.h"
  Delete "$INSTDIR\include\t_name.h"
  Delete "$INSTDIR\include\y_fcth.h"
  Delete "$INSTDIR\include\fi_lib_consts.hpp"
  Delete "$INSTDIR\include\fi_lib.hpp"
  Delete "$INSTDIR\include\xscclass.hpp"
  Delete "$INSTDIR\include\cdot.hpp"
  Delete "$INSTDIR\include\cidot.hpp"
  Delete "$INSTDIR\include\cimath.hpp"
  Delete "$INSTDIR\include\cimatrix.hpp"
  Delete "$INSTDIR\include\cinterval.hpp"
  Delete "$INSTDIR\include\civeccmat.hpp"
  Delete "$INSTDIR\include\civecimat.hpp"
  Delete "$INSTDIR\include\civecrmat.hpp"
  Delete "$INSTDIR\include\civector.hpp"
  Delete "$INSTDIR\include\cmatimat.hpp"
  Delete "$INSTDIR\include\cmatrix.hpp"
  Delete "$INSTDIR\include\compiler.h"
  Delete "$INSTDIR\include\complex.hpp"
  Delete "$INSTDIR\include\cvecimat.hpp"
  Delete "$INSTDIR\include\cvecrmat.hpp"
  Delete "$INSTDIR\include\cvector.hpp"
  Delete "$INSTDIR\include\cxscmatr.hpp"
  Delete "$INSTDIR\include\cxscvect.hpp"
  Delete "$INSTDIR\include\dot.hpp"
  Delete "$INSTDIR\include\dot_defs.hpp"
  Delete "$INSTDIR\include\except.hpp"
  Delete "$INSTDIR\include\idot.hpp"
  Delete "$INSTDIR\include\imath.hpp"
  Delete "$INSTDIR\include\imatrix.hpp"
  Delete "$INSTDIR\include\interval.hpp"
  Delete "$INSTDIR\include\intmatrix.hpp"
  Delete "$INSTDIR\include\intvector.hpp"
  Delete "$INSTDIR\include\ioflags.hpp"
  Delete "$INSTDIR\include\iveccmat.hpp"
  Delete "$INSTDIR\include\iveccvec.hpp"
  Delete "$INSTDIR\include\iveclrmat.hpp"
  Delete "$INSTDIR\include\ivecrmat.hpp"
  Delete "$INSTDIR\include\ivector.hpp"
  Delete "$INSTDIR\include\l_complex.hpp"
  Delete "$INSTDIR\include\l_imath.hpp"
  Delete "$INSTDIR\include\l_imatrix.hpp"
  Delete "$INSTDIR\include\l_interval.hpp"
  Delete "$INSTDIR\include\l_ivector.hpp"
  Delete "$INSTDIR\include\l_real.hpp"
  Delete "$INSTDIR\include\l_rmath.hpp"
  Delete "$INSTDIR\include\l_rmatrix.hpp"
  Delete "$INSTDIR\include\l_rvector.hpp"
  Delete "$INSTDIR\include\livecimat.hpp"
  Delete "$INSTDIR\include\liveclrmat.hpp"
  Delete "$INSTDIR\include\livecrmat.hpp"
  Delete "$INSTDIR\include\lrmatimat.hpp"
  Delete "$INSTDIR\include\lrvecimat.hpp"
  Delete "$INSTDIR\include\lrvecivec.hpp"
  Delete "$INSTDIR\include\lrvecrmat.hpp"
  Delete "$INSTDIR\include\matrix.hpp"
  Delete "$INSTDIR\include\real.hpp"
  Delete "$INSTDIR\include\rmath.hpp"
  Delete "$INSTDIR\include\rmatrix.hpp"
  Delete "$INSTDIR\include\rts_real.hpp"
  Delete "$INSTDIR\include\RtsFunc.h"
  Delete "$INSTDIR\include\rtsrmath.h"
  Delete "$INSTDIR\include\RtsTyp.h"
  Delete "$INSTDIR\include\rvector.hpp"
  Delete "$INSTDIR\include\test.hpp"
  Delete "$INSTDIR\include\testclss.hpp"
  Delete "$INSTDIR\include\testcomp.hpp"
  Delete "$INSTDIR\include\testdot.hpp"
  Delete "$INSTDIR\include\testintv.hpp"
  Delete "$INSTDIR\include\testmatr.hpp"
  Delete "$INSTDIR\include\testsklr.hpp"
  Delete "$INSTDIR\include\testvect.hpp"
  Delete "$INSTDIR\include\vector.hpp"
  Delete "$INSTDIR\include\cxscconf.h"
  Delete "$INSTDIR\include\stacksz.hpp"
  Delete "$INSTDIR\include\ci_util.hpp"
  Delete "$INSTDIR\include\cipoly.hpp"
  Delete "$INSTDIR\include\cpoly.hpp"
  Delete "$INSTDIR\include\cpzero.hpp"
  Delete "$INSTDIR\include\ddf_ari.hpp"
  Delete "$INSTDIR\include\expreval.hpp"
  Delete "$INSTDIR\include\gop1.hpp"
  Delete "$INSTDIR\include\gop.hpp"
  Delete "$INSTDIR\include\grad_ari.hpp"
  Delete "$INSTDIR\include\hess_ari.hpp"
  Delete "$INSTDIR\include\i_util.hpp"
  Delete "$INSTDIR\include\linsys.hpp"
  Delete "$INSTDIR\include\lop.hpp"
  Delete "$INSTDIR\include\lop_ari.hpp"
  Delete "$INSTDIR\include\lst1_ari.hpp"
  Delete "$INSTDIR\include\lst_ari.hpp"
  Delete "$INSTDIR\include\matinv.hpp"
  Delete "$INSTDIR\include\mv_util.hpp"
  Delete "$INSTDIR\include\mvi_util.hpp"
  Delete "$INSTDIR\include\nlfzero.hpp"
  Delete "$INSTDIR\include\nlinsys.hpp"
  Delete "$INSTDIR\include\r_util.hpp"
  Delete "$INSTDIR\include\rev_simp.hpp"
  Delete "$INSTDIR\include\rpeval.hpp"
  Delete "$INSTDIR\include\rpoly.hpp"
  Delete "$INSTDIR\include\set_ari.hpp"
  Delete "$INSTDIR\include\*.hpp"
  Delete "$INSTDIR\examples\test\ergebnis.dat"
  Delete "$SMPROGRAMS\cxsc\Uninstall.lnk"
  Delete "$SMPROGRAMS\cxsc\Website.lnk"
  Delete "$INSTDIR\examples\test\*.*"
  Delete "$INSTDIR\examples\project\myexample\*.*"
  Delete "$INSTDIR\examples\project\*.*"

  RMDir "$SMPROGRAMS\cxsc"
  RMDir "$INSTDIR\lib"
  RMDir "$INSTDIR\include"
  RMDir "$INSTDIR\examples\test"
  RMDir "$INSTDIR\examples\project\myexample"
  RMDir "$INSTDIR\examples\project\"
  RMDir "$INSTDIR\examples"
  RMDir "$INSTDIR"

  DeleteRegKey ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}"
  DeleteRegKey HKLM "${PRODUCT_DIR_REGKEY}"
  SetAutoClose true
SectionEnd

Section "Add to path"
  Push $INSTDIR
  Call AddToPath

  Push "LIB"
  Push "$INSTDIR\lib"
  Call AddToEnvVar

  Push "INCLUDE"
  Push "$INSTDIR\include"
  Call AddToEnvVar
SectionEnd
