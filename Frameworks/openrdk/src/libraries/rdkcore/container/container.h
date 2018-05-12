/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef RDK2_CONTAINERS_VECTOR
#define RDK2_CONTAINERS_VECTOR

#include <vector>
#include <list>
#include <string>

#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/object/objectdiff.h>

namespace RDK2 { namespace Containers {

	using namespace std;
	
	/**
	
	Il vero container e' un membro: e' piu' facile fare tutto in modo corretto.
	FIXME DC: che vuol dire membro? membro di che?
	
	Gli oggetti dentro devono implementare anche clone().
	
	*/

// 	template<class RType>
// 	class Vector;
// 	
// 	template<class RType>
// 	class VectorDiff: public RDK2::Meta::ObjectDiff {
// 	public:
// 		VectorDiff() : RDK2::Meta::ObjectDiff("Vector") { }
// 		vector<RDK2::Meta::ObjectDiff*> split(size_t) { return vector<RDK2::Meta::ObjectDiff*>(); }
// 	
// 		void read(RDK2::Reader*r) throw (RDK2::ReadingException) {
// 			r->startReading(getClassName());
// 			uint sz = r->read_i32("size");
// 			for (size_t i = 0; i < sz; i++) {
// 				Object* ob = (Object*) r->readObject();
// 				RType* ob2 = dynamic_cast<RType*>(ob);
// 				if (!ob2) r->error("Foreign object");
// 				v.push_back(ob2);
// 			}
// 			r->doneReading();
// 		}
// 		
// 		void write(RDK2::Writer*w) const throw (RDK2::WritingException) {
// 			w->startWriting(getClassName());
// 			w->write_i32(v.size(), "size");
// 			for (size_t i = 0; i < v.size(); i++) {
// 				RType* ob = v[i];
// 				w->writeObject(true, ob);
// 			}
// 			w->doneWriting();
// 		}
// 	
// 	public:
// 		vector<RType*> v;
// 		uint remoteIdx;
// 	};

	template<class RType>
	class Vector: public RDK2::Meta::ObjectDiff /* and thus also public RDK2::Object */
	{
		protected:
			uint remoteIdx;
		
		public:
		string objsClassName;

		public: // FIXME it MUST be at least protected
		// DC: e perch� has-a e non is-a ?
		std::vector<RType*> c;
	
		Vector() : ObjectDiff("Vector"), remoteIdx(0) { RType rt; objsClassName = rt.getClassName(); }
		
		Vector& operator=(const Vector<RType>& c2) {
			return copy<RType>(c2);
		}
			
		template<class R2>
		Vector& copy(const Vector<R2>& c2) {
			clear();
			return append(c2);
		}
		
		template<class R2>
		Vector& append(const Vector<R2>& c2) {
			objsClassName = c2.objsClassName;
			for(typename Vector<R2>::const_iterator i=c2.begin();i!=c2.end();++i) {
				const R2 * r2 = *i;
				const RType * r1 = dynamic_cast<const RType*>(r2);
				if(!r1) continue;
				RDK2::Object * clone = r1->clone();
				if(!clone) continue;
				RType *clone2 = dynamic_cast<RType*>(clone);
				if(!clone2) { delete clone; continue; }
				push_back(clone2);
			}
			return *this;
		}
		
		RDK2::Object* clone() const {
			return new Vector<RType>(*this);
		}

		Vector(const Vector<RType>& c2) : 
			RDK2::Meta::ObjectDiff("Vector"), remoteIdx(c2.remoteIdx)
		{
			copy<RType>(c2);
		}
		
		virtual ~Vector() {
			clear();
		}
	
		typedef typename std::vector<RType*>::iterator iterator;
		typedef typename std::vector<RType*>::const_iterator const_iterator;
		
		inline iterator begin() { return c.begin(); }
		inline const_iterator begin() const { return c.begin(); }
		inline iterator end() { return c.end(); }
		inline const_iterator end() const { return c.end(); }

		inline size_t size() const { return c.size(); }
		
		inline void resize(size_t n){ c.resize(n);}
		
		inline void reserve(size_t n){ c.reserve(n);}

		RType* operator[](uint i) const { return c[i]; }
		RType* getItem(uint i) const { return c[i]; }

		template<typename T>
		T* getItemAs(uint i) const { return dynamic_cast<T*>(c[i]); }	
	
		string getClassName() const { return objsClassName + "Vector"; }

		void push_back(RType*t) { c.push_back(t); }
	
		iterator erase(iterator it, bool alsoDelete) {
			RType* rt = *it;
			iterator retIt = c.erase(it);
			if (alsoDelete) delete rt;
			return retIt;
		}

		void clear(bool alsoDelete) {
			for (iterator it = c.begin(); it != c.end(); ) {
				it = erase(it, alsoDelete);
			}
		}
	
		void clear() {
			for(iterator i=c.begin();i!=c.end();i++) {
				RType * object = *i;
				delete object;
			}
			c.clear();
		}
		
		virtual void read(Reader*r) throw (ReadingException) {
			uint vers = r->startReading(getClassName());
			int size = r->read_i32("size");
			if (vers >= 2) remoteIdx = r->read_i32("remoteIdx");
			else remoteIdx = 0;
			bool special = r->read_u8("special");
			if(special) {
				string same = r->readString("class");
				if(!supportedSpecial(same)) 
					r->error( string("Unsupported special: ") + same);
#ifndef SWIGRUBY				
				if(same=="RString") {
					for(int i=0;i<size;i++) {
						string s = r->readString();
						RDK2::RPrimitive::RString * rs = new RDK2::RPrimitive::RString(s);
						
						RType * r2 = dynamic_cast<RType*>(rs);
						if(r2) {
							c.push_back(r2);
						} else {
							delete rs;
							r->error("Attempting to put RStrings into other container");
						}
					}
				}
#endif				
			} else {
				//std::cout << "Reading " << size << " objects..." << std::endl;
				for(int i=0;i<size;i++) {
					Object * ob = (Object*) r->readObject();
					RType * ob2 = dynamic_cast<RType*>(ob);
					if(!ob2) r->error("Foreign object");
					//std::cout << "read " << ob2->getStringRepresentation() << endl;
					c.push_back(ob2);
				}
				
			}
			r->doneReading();
		}
		
		bool sameClass(string&cl) const {
			if(c.size()==0) return false;
			
			cl = c[0]->getClassName();
			for(int i=1;i<(int)c.size();i++)
				if(c[i]->getClassName() != cl)
					return false;
			
			return true;
		}
		
		bool supportedSpecial(cstr s) const {
			if(s=="RString") return true;
			return false;
		}
		
		virtual void write(Writer*w) const throw (WritingException) {
			w->startWriting(getClassName(), 2);
				w->write_i32(c.size(), "size");
				w->write_i32(remoteIdx, "remoteIdx");

				string same;
				if(sameClass(same) && supportedSpecial(same)) {
					w->write_u8(true, "special");
					w->writeString(same, "class");
#ifndef SWIGRUBY
					if(same=="RString") {
						for(const_iterator i=c.begin();i!=c.end();++i) {
							const RDK2::RPrimitive::RString * ob = 
								dynamic_cast<const RDK2::RPrimitive::RString*>(*i);
							w->writeString(ob->value);
						}
					}
#endif
				} else {
					w->write_u8(false, "special");
					for(const_iterator i=c.begin();i!=c.end();++i) {
						RType * ob = *i;
						w->writeObject(true, ob);
					}
				}
			w->doneWriting();
		}
	
		virtual bool knowsDiffs() { return true; }
		
		// FIXME DC: penso che questa funzione non sia utile a nulla
		// ma per ora serve come interfaccia di ObjectDiff
		virtual vector<RDK2::Meta::ObjectDiff*> split(size_t) {
			return vector<RDK2::Meta::ObjectDiff*>(); }
		
		virtual bool applyDiff(const RDK2::Meta::ObjectDiff* od) {
			const Vector<RType>* vd = dynamic_cast<const Vector<RType>*>(od);
			if (!vd) return false;
			else if (vd->remoteIdx < remoteIdx) return true;	// this diff is older than us
			else {
				if (vd->remoteIdx > remoteIdx) {
					remoteIdx = vd->remoteIdx;
/*					cout << "Arrived a remoteIdx "<<remoteIdx;
					cout <<", I contained "<<size()<<"elements"<<endl;*/
					clear(true);
				}
				for (size_t i = 0; i < vd->size(); i++) {
					push_back((RType*) vd->operator[](i)->clone());
				}
			}
			return true;
		}
		
		virtual vector<RDK2::Meta::ObjectDiff*> splitInDiffs(size_t) {
			vector<RDK2::Meta::ObjectDiff*> ret;
			Vector<RType>* vd = new Vector<RType>;
			vd->remoteIdx = remoteIdx;
			for (size_t i = 0; i < c.size(); i++) {
				vd->push_back((RType*) c[i]->clone());	// the object is cloned
				if ((i % 30) == 0 || i == c.size()-1) { // FIXME
					ret.push_back(vd);	// now the objects are not cloned
					if (i != c.size()-1) {
						vd = new Vector<RType>; vd->remoteIdx = remoteIdx;
					}
				}
			}
			remoteIdx++;
			return ret;
		}
	};

}} // ns

namespace RDK2 { namespace Containers {
	typedef RDK2::Containers::Vector<RDK2::Object> RObjectVector;
}}

#endif
