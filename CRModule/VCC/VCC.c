/*****************************************************************************
 *
 *              VCC.c -- VCC v0.1
 *
 *****************************************************************************
 *
 * Author:          Guillermo Jara Luengos
 * FileName:        VCC.c
 * Dependencies:    VCC.h
 * Processor:
 * BOARD:
 * Compiler:        C32 01.00.02 or higher
 * Linker:          MPLINK 03.40.00 or higher
 * Company:         B105 -FAILURE IS NOT AN OPTION-
 *
 *****************************************************************************
 * File Description:
 *
 * Este archivo implementa funciones �tiles para tener una Virtual Control
 * Channel b�sico.
 *
 * Change History:
 *  Rev   Date(m/d/y)   	Description
 *  0.1   February 6, 2013, 12:34 PM    Initial revision
 *****************************************************************************/

#include "CRModule/VCC/VCC.h"

/*****************************************************************************/
/*********************************VARIABLES***********************************/
/*****************************************************************************/
//BYTE BufferTx[TX_BUFFER_SIZE];
//BYTE BufferRx[RX_BUFFER_SIZE];

/*****************************************************************************/
/******************************FIN VARIABLES**********************************/
/*****************************************************************************/

/*****************************************************************************/
/*********************************FUNCIONES***********************************/
/*****************************************************************************/

/*De recepcion/interfaz con Messenger*/
//Para recibir podemos llamar a la funcion con cualquier direccion destino.
BOOL CRM_VCC_Mssg_Rcvd(VCC_MSSG_RCVD *Peticion)
{
    switch((*Peticion).Action)
    {
        case(ActSend):
            CRM_VCC_Sender(Peticion);
            break;
        case(ActRecv):
            CRM_VCC_Reciever(Peticion);
            break;
        default:
            break;
    }
    return TRUE;
}
/*Fin de funciones de recepcion/interfaz con Messenger*/


//De Funcionalidad propia del sub-m�dulo.
/*La rutina para gestionar la recepcion de mensajes por el canal del VCC. Habr�
 por tanto mensajes que ser�n realmente de control para los m�dulos o quiz�
 opcionalmente se pueda a�adir la posibilidad de enviar mensajes de datos que se
 enviar�an a la aplicaci�n*/
BOOL CRM_VCC_Reciever(VCC_MSSG_RCVD *Peticion)
{
    VCCMSSGTYPE CtrlHeader;
#if defined DATA_OVER_VCC
//    if(!DatosDeApp) //XXX-GuilJa si hay datos de app no hacemos nada y esperamos a que este libre.
//    {
#endif
//        if(GuilJa_MiWi_Rcvd_Buffer(Peticion->BufferVCC, Peticion->Param1))
        if(CtrlMssgFlag)
        {
//            CtrlHeader = (VCCMSSGTYPE)((RECEIVED_MESSAGE*)Peticion->BufferVCC)->Payload[CtrlMssgField]; /*Al inicio del mensaje
//                                                             recibido.*/
//            switch(CtrlHeader)
//            {
//                case 0x00:
//                    //Los mensajes de control de MiWi tienen esta cabecera.
//                    break;
//                case VccCtrlMssg: /*Si efectivamente es un mensaje para el CRModule lo
//                            pasamos por el discretizador para que genere un mensaje
//                            para Messenger y que el procese la peticion.*/
//    //                CRM_VCC_MssgCreator(((RECEIVED_MESSAGE*)Peticion->BufferVCC)->Payload+1);
//                    CRM_VCC_MssgCreator((RECEIVED_MESSAGE*)(Peticion->BufferVCC));
//                    /*Le hemos sumado uno al puntero del buffer para no pasarle la
//                     cabecera propia de mensaje de control que hemos filtrado aqui.*/
//                    break;
//                default:
//                    //TODO es un mensaje de datos.
//                    DatosDeApp = TRUE;
//                    break;
//            }
            CRM_VCC_MssgCreator((RECEIVED_MESSAGE*)(Peticion->BufferVCC));
            CtrlMssgFlag = FALSE;
        }
#if defined DATA_OVER_VCC
//    }
#endif

}

/*La rutina para enviar mensajes por el VCC hacia otros m�dulos. Construye los
 mensajes con los campos necesarios.*/
BOOL CRM_VCC_Sender(VCC_MSSG_RCVD *Peticion)
{
    BOOL salida;
    //TODO REVISAR la composicion del mensaje saliente.
    salida = Send_Buffer(Peticion->Transceiver, Peticion->BufferVCC, Peticion->DirNodDest);
    return salida;
}
//Fin de funciones de funcionalidad propia del sub-m�dulo.

//Funciones de funcionalidad interna del sub-modulo.*/
/*Para generar un message apropiado para mandarle al messenger.*/
BOOL CRM_VCC_MssgCreator(INPUT RECEIVED_MESSAGE *Buffer)
{
    //TODO en vez de optm aqui tiene que ser del messenger.
    MSN_MSSG_RCVD PeticionGeneral;
//    void* Peticion4SubMDest;
    DISC_MSSG_RCVD Peticion4Disc;
    EXEC_MSSG_RCVD Peticion4Exec;
//    OPTM_MSSG_RCVD Peticion4Optm;
    REPO_MSSG_RCVD Peticion4Repo;
    POLI_MSSG_RCVD Peticion4Poli;
    ACCCTRL_MSSG_RCVD Peticion4AccCtrl;
    switch(Buffer->Payload[SubMDestField])
    {
        case SubM_Disc: /*Para Discovery.*/
            CRM_Message(VCC, SubM_Disc, Buffer + 1);
            break;
        case SubM_Exec: /*Para Executor.*/
            CRM_Message(VCC, SubM_Exec, Buffer + 1);
            break;
        case SubM_Opt: /*Para Optmimizer.*/
        {
            /*Preparamos el Message (para Messenger) y que embeba la peticion para
             el sub-modulo destino.*/
            OPTM_MSSG_RCVD Peticion4Optm;
//            Peticion4SubMDest = &Peticion4Optm;
//            PeticionGeneral.Peticion_Destino.PeticionOptm = Peticion4SubMDest;
            PeticionGeneral.Peticion_Destino.PeticionOptm = &Peticion4Optm;
            PeticionGeneral.DireccionEUI = Buffer->SourceAddress;
            Peticion4Optm.Action = Buffer->Payload[SubMDestActField];
            Peticion4Optm.Param1 = Buffer->Payload + SubMOptParam1Field;
            Peticion4Optm.Param2 = Buffer->Payload + SubMOptParam2Field;
            Peticion4Optm.Transceiver = Buffer->Payload[SubMOptParamTransceiver];
            Peticion4Optm.EUINodo = Buffer->SourceAddress;
//            ((OPTM_MSSG_RCVD*)Peticion4SubMDest)->Action = Buffer->Payload[SubMDestActField];
//            ((OPTM_MSSG_RCVD*)Peticion4SubMDest)->Param1 = Buffer->Payload + SubMOptParam1Field;
//            ((OPTM_MSSG_RCVD*)Peticion4SubMDest)->Param2 = Buffer->Payload + SubMOptParam2Field;
//            ((OPTM_MSSG_RCVD*)Peticion4SubMDest)->EUINodo = Buffer->Payload + SubMOptEUIField; //Esto lo ten�a as� en el TEST5
//            ((OPTM_MSSG_RCVD*)Peticion4SubMDest)->EUINodo = Buffer->SourceAddress;
            /*Y enviamos el Message al sub-modulo (a traves de Messenger.*/
            CRM_Message(VCC, SubM_Opt, &PeticionGeneral);
            break;
        }
        case SubM_Repo: /*Para Repository.*/
            CRM_Message(VCC, SubM_Repo, Buffer + 1);
            break;
        case SubM_Poli: /*Para Policies.*/
            CRM_Message(VCC, SubM_Poli, Buffer + 1);
            break;
        case SubM_AccCtrl: /*Para Access Control.*/
            CRM_Message(VCC, SubM_AccCtrl, Buffer + 1);
            break;
        default:
            break;            
    }
}
//Fin de funciones de funcionalidad interna.

/*****************************************************************************/
/******************************FIN DE FUNCIONES*******************************/
/*****************************************************************************/
