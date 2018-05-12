/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef H_RDK2_PROFILING
#define H_RDK2_PROFILING


/*

Modulo di profiling
===================

Questo modulo scrive su file le informazioni necessarie al profiling dell'RDK.
Un programma a parte legge dal file e visualizza statistiche.

Interfaccia dall'RDK
====================

Per i moduli: avranno una comoda Module::eventStart(name) e Module::eventEnd(name)
 che chiameranno Manager::start() e Manager::end() definite sotto.
 
Per le code: un thread periodico (1 sec?) ad alta priorit� scrive lo stato delle
 code tramite la Manager::addCustomLine().

Formato dei messaggi di profiling
=================================

Messaggi per stimare durata eventi:

	START <event_name>  <time_now>  <clock>
	END   <event_name>  <time_now>  <clock>

dove: time_now � gettimeofday(), <clock> � clock(), tutto scritto con interi 
(non con i double). <event_name> per i task � del tipo NomeTask::NomeEvento.
	
Altri messaggi:

	PUSH <nome> <time_now> <lunghezza> <time_last_push> <time_last_freeze>

	FREEZE <nome coda> <sessione> <time_now> <lunghezza>

	

Il caso tipico di uso

void exec() {
	event_start("miotask")

	event_start("miotask:sm")
	scana_matching();
	event_end("miotask:sm")
	
	event_start("oc")
	occupancy_grid();
	event_end("oc")
	
	event_end("miotask")
}

Cose dell'analizzatore:
- vedere eventi cominciati e non finiti, in particolare vedere il primo
- grafico tempo, tempo impiegato
- accorgersi se qualcuno non ha fatto freeze da un po'
- grafico andamento dimensione coda, tempo di processamento per oggetto
*/

namespace RDK2 { namespace Profiling {

	struct Profiler {
	
		/** Apre il file per il profiling */
		static void open(const char* filename);
	
		/** Scrive un messaggio di tipo START */
		static void start(const char* name);
		
		/** Scrive un messaggio di tipo END */
		static void end(const char* name);
	
		/** Scrive un messaggio di tipo LOCK */
		static void lock(const char* name);
		
		/** Scrive un messaggio di tipo UNLOCK */
		static void unlock(const char* name);
		
		/** Scrive un messaggio custom - usato ad esempio per le code */
		static void addCustomLine(const char* what, const char* name, const char* otherInfo = "");
	};
	
}} // namespace RDK2::Profiling

#endif
