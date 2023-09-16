/**
 * @file tpl_dispatch_table.c
 *
 * @section desc File description
 *
 * Dispatch table for application ECM
 * Automatically generated by goil on Sat Sep 16 18:43:52 2023
 * from root OIL file ACC_ECU.oil
 *
 * @section copyright Copyright
 *
 * Trampoline OS
 *
 * Trampoline is copyright (c) IRCCyN 2005-2010
 * Trampoline is protected by the French intellectual property law.
 *
 * This software is distributed under the Lesser GNU Public Licence
 *
 * @section infos File informations
 *
 * $Date$
 * $Rev$
 * $Author$
 * $URL$
 */

#include "tpl_dispatch_table.h"

#include "tpl_os_os_kernel.h"
#include "tpl_os_os.h"
#include "tpl_os_interrupt_kernel.h"
#include "tpl_os_task_kernel.h"
#include "tpl_os_resource_kernel.h"
#include "tpl_os_alarm_kernel.h"

#define OS_START_SEC_CONST_UNSPECIFIED
#include "tpl_memmap.h"
CONST(tpl_system_call, OS_CONST) tpl_dispatch_table[SYSCALL_COUNT] = {
/* os services */
    (tpl_system_call) tpl_resume_os_interrupts_service,
    (tpl_system_call) tpl_enable_all_interrupts_service,
    (tpl_system_call) tpl_suspend_os_interrupts_service,
    (tpl_system_call) tpl_disable_all_interrupts_service,
    (tpl_system_call) tpl_call_terminate_isr2_service,
    (tpl_system_call) tpl_suspend_all_interrupts_service,
    (tpl_system_call) tpl_call_terminate_task_service,
    (tpl_system_call) tpl_resume_all_interrupts_service,
    (tpl_system_call) tpl_terminate_task_service,
    (tpl_system_call) tpl_start_os_service,
    (tpl_system_call) tpl_shutdown_os_service,
    (tpl_system_call) tpl_get_resource_service,
    (tpl_system_call) tpl_activate_task_service,
    (tpl_system_call) tpl_release_resource_service,
    (tpl_system_call) tpl_chain_task_service,
    (tpl_system_call) tpl_get_alarm_base_service,
    (tpl_system_call) tpl_get_task_id_service,
    (tpl_system_call) tpl_get_alarm_service,
    (tpl_system_call) tpl_get_active_application_mode_service,
    (tpl_system_call) tpl_set_rel_alarm_service,
    (tpl_system_call) tpl_schedule_service,
    (tpl_system_call) tpl_set_abs_alarm_service,
    (tpl_system_call) tpl_cancel_alarm_service,
    (tpl_system_call) tpl_get_task_state_service
};


#define OS_STOP_SEC_CONST_UNSPECIFIED
#include "tpl_memmap.h"

/* End of file tpl_dispatch_table.c */

