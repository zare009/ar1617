#include <iostream>
#include <set>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <memory>
#include <functional>
#include <list>

using namespace std;

/* Funkcijski i predikatski simboli */
typedef string FunctionSymbol;
typedef string PredicateSymbol;

/* Signatura se sastoji iz funkcijskih i predikatskih simbola kojima
   su pridruzene arnosti (nenegativni celi brojevi) */
class Signature {
private:
  map<FunctionSymbol,  unsigned> _functions;
  map<PredicateSymbol, unsigned> _predicates;
public:
  
  /* Dodavanje funkcijskog simbola date arnosti */
  void addFunctionSymbol(const FunctionSymbol & f, unsigned arity);

  /* Dodavanje predikatskog simbola date arnosti */
  void addPredicateSymbol(const PredicateSymbol & p, unsigned arity);

  /* Provera da li postoji dati funkcijski simbol, i koja mu je arnost */
  bool checkFunctionSymbol(const FunctionSymbol & f, unsigned & arity) const;

  /* Provera da li postoji dati predikatski simbol, i koja mu je arnost */
  bool checkPredicateSymbol(const PredicateSymbol & f, unsigned & arity) const;
  
};

/* Tip podatka za predstavljanje varijable */
typedef string Variable;

/* Skup varijabli */
typedef set<Variable> VariableSet;


class Structure; // L-strukture (videti dole) 
class Valuation; // Valuacija (videti dole)

class BaseTerm;
typedef std::shared_ptr<BaseTerm> Term;


/* Apstraktna klasa BaseTerm koja predstavlja termove */
class BaseTerm : public enable_shared_from_this<BaseTerm> {

public:
  /* Termovi mogu biti ili varijable ili funkcijski simboli primenjeni
     na (0 ili vise) termova */
  enum Type { TT_VARIABLE, TT_FUNCTION };

  /* Vraca tip terma */
  virtual Type getType() const = 0;

  /* Prikazuje term */
  virtual void printTerm(ostream & ostr) const = 0;

  /* Ispituje sintaksnu jednakost termova */
  virtual bool equalTo(const Term & t) const = 0;
  
  /* Vraca skup svih varijabli koje se pojavljuju u termu */
  virtual void getVars(VariableSet & vars) const = 0;
 
  /* Odredjuje da li se data varijabla nalazi u termu */
  bool containsVariable(const Variable & v) const;

  /* Odredjuje interpretaciju terma u datoj L-strukturi i datoj valuaciji */
  virtual unsigned eval(const Structure & st, const Valuation & val) const = 0;

  /* Zamena varijable v termom t */
  virtual Term substitute(const Variable & v, const Term & t) = 0;

  virtual ~BaseTerm() {}
};


ostream & operator << (ostream & ostr, const Term & t);

/* Term koji predstavlja jednu varijablu */
class VariableTerm : public BaseTerm {
private:
  Variable _v;
public:
  VariableTerm(const Variable & v);
  virtual Type getType() const;
  const Variable & getVariable() const;
  virtual void printTerm(ostream & ostr) const;
  virtual bool equalTo(const Term & t) const;
  virtual void getVars(VariableSet & vars) const; 
  virtual unsigned eval(const Structure & st, const Valuation & val) const;
  virtual Term substitute(const Variable & v, const Term & t);
};


/* Term koji predstavlja funkcijski simbol primenjen na odgovarajuci
   broj podtermova */
class FunctionTerm : public BaseTerm {
private:
  const Signature & _sig;
  FunctionSymbol _f;
  vector<Term> _ops;

public:
  FunctionTerm(const Signature & s, const FunctionSymbol & f, 
	       const vector<Term> & ops);
  FunctionTerm(const Signature & s, const FunctionSymbol & f, 
	       vector<Term> && ops = vector<Term> ());
  virtual Type getType() const;
  const Signature & getSignature() const;
  const FunctionSymbol & getSymbol() const;
  const vector<Term> & getOperands() const;
  virtual void printTerm(ostream & ostr) const;
  virtual bool equalTo(const Term & t) const;
  virtual void getVars(VariableSet & vars) const;
  virtual unsigned eval(const Structure & st, const Valuation & val) const;
  virtual Term substitute(const Variable & v, const Term & t); 
};


class BaseFormula;
typedef std::shared_ptr<BaseFormula> Formula;

/* Apstraktna klasa kojom se predstavljaju formule */
class BaseFormula : public enable_shared_from_this<BaseFormula> {
  
public:

  /* Tipovi formula (dodatak u odnosu na iskaznu logiku su formule 
     kod kojih je vodeci simbol univerzalni ili egzistencijalni
     kvantifikator */
  enum Type { T_TRUE, T_FALSE, T_ATOM, T_NOT, 
	      T_AND, T_OR, T_IMP, T_IFF, T_FORALL, T_EXISTS };
  
  /* Prikaz formule */ 
  virtual void printFormula(ostream & ostr) const = 0;
  
  /* Tip formule */
  virtual Type getType() const = 0;
  
  /* Slozenost formule */
  virtual unsigned complexity() const = 0;

  /* Sintaksna jednakost dve formule */
  virtual bool equalTo(const Formula & f) const = 0;

  /* Ocitava sve varijable koje se pojavljuju u formuli. Ako
     je zadat drugi parametar sa vrednoscu true, tada se izdvajaju samo
     slobodne varijable u formuli */
  virtual void getVars(VariableSet & vars, bool free = false) const = 0;

  /* Ispituje da li se varijabla pojavljuje u formuli (kao slobodna ili
     vezana) */
  bool containsVariable(const Variable & v, bool free = false) const;

  /* Izracunava interpretaciju formule za datu L-strukturu i valuaciju */
  virtual bool eval(const Structure & st, const Valuation & val) const = 0;

  /* Zamena slobodnih pojavljivanja varijable v termom t */
  virtual Formula substitute(const Variable & v, const Term & t) = 0;

  /* Funkcija simplifikuje formulu (uklanja konstante i nepotrebne 
     kvantifikatore) */
  virtual Formula simplify() = 0;

  /* Funkcija svodi formulu na NNF */
  virtual Formula nnf() = 0;

  /* Pomocna funkcija za izdvajanje kvantifikatora */
  virtual Formula pullquants() = 0;

  /* Funkcija svodi formulu na PRENEX */
  virtual Formula prenex() = 0;

  /* Funkcija za skolemizaciju */
  virtual Formula skolem(Signature & s, vector<Variable> && vars = vector<Variable>());
  
  virtual ~BaseFormula() {}
};

ostream & operator << (ostream & ostr, const Formula & f);

/* Funkcija vraca novu varijablu koja se ne pojavljuje ni u e1 ni u e2 */
template <typename T1, typename T2>
Variable getUniqueVariable(const T1 & e1, const T2 & e2);

/* Funkcija vraca novi funkcijski simbol koji se ne pojavljuje u formuli */
FunctionSymbol getUniqueFunctionSymbol(const Signature & s);


/* Klasa predstavlja sve atomicke formule (True, False i Atom) */
class AtomicFormula : public BaseFormula {

public:
  virtual unsigned complexity() const;
  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};

/* Klasa predstavlja logicke konstante (True i False) */
class LogicConstant : public AtomicFormula {

public:
  virtual bool equalTo(const Formula & f) const;
  virtual void getVars(VariableSet & vars, bool free) const;
  virtual Formula substitute(const Variable & v, const Term & t);
};

/* Klasa predstavlja True logicku konstantu */
class True : public LogicConstant {

public:
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
};

/* Klasa predstavlja logicku konstantu False */
class False : public LogicConstant {

public:
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const; 
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
};

/* Klasa predstavlja atom, koji za razliku od iskazne logike ovde ima
   znatno slozeniju strukturu. Svaki atom je predikatski simbol primenjen
   na odgovarajuci broj podtermova */
class Atom : public AtomicFormula {
private:
  const Signature & _sig;
  PredicateSymbol _p;
  vector<Term> _ops;

public:
  Atom(const Signature & s, 
       const PredicateSymbol & p, 
       const vector<Term> & ops);
  Atom(const Signature & s, 
       const PredicateSymbol & p, 
       vector<Term> && ops = vector<Term>());
  const PredicateSymbol & getSymbol() const;
  const Signature & getSignature() const;
  const vector<Term> & getOperands() const;
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const; 
  virtual bool equalTo(const Formula & f) const;
 
  virtual void getVars(VariableSet & vars, bool free) const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);
};


/* Klasa unarni veznik (obuhvata negaciju) */
class UnaryConnective : public BaseFormula {
protected:
  Formula _op;
public:
  UnaryConnective(const Formula & op);
  const Formula & getOperand() const;
  virtual unsigned complexity() const;
  virtual bool equalTo(const Formula & f) const;
  virtual void getVars(VariableSet & vars, bool free) const;
};

/* Klasa koja predstavlja negaciju */
class Not : public UnaryConnective {
public:

  using UnaryConnective::UnaryConnective;
  virtual void printFormula(ostream & ostr) const; 
  virtual Type getType() const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};

/* Klasa predstavlja sve binarne veznike */
class BinaryConnective : public BaseFormula {
protected:
   Formula _op1, _op2;
public:
  BinaryConnective(const Formula & op1, const Formula & op2);
  const Formula & getOperand1() const;
  const Formula & getOperand2() const;
  virtual unsigned complexity() const;
  virtual bool equalTo(const Formula & f) const;
  virtual void getVars(VariableSet & vars, bool free) const;
};

/* Klasa predstavlja konjunkciju */
class And : public BinaryConnective {
public:
  using BinaryConnective::BinaryConnective;
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const; 
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t); 

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};


/* Klasa predstavlja disjunkciju */
class Or : public BinaryConnective {
public:
  using BinaryConnective::BinaryConnective;
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};

/* Klasa predstavlja implikaciju */
class Imp : public BinaryConnective {
public:
  using BinaryConnective::BinaryConnective;
  virtual void printFormula(ostream & ostr) const;
  virtual Type getType() const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};


/* Klasa predstavlja ekvivalenciju */
class Iff : public BinaryConnective {

public:
  using BinaryConnective::BinaryConnective;
  virtual void printFormula(ostream & ostr) const; 
  virtual Type getType() const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();

};

/* Klasa predstavlja kvantifikovane formule */
class Quantifier : public BaseFormula {
protected:
  Variable _v;
  Formula _op;

public:
  Quantifier(const Variable & v, const Formula & op);
  const Variable & getVariable() const;
  const Formula & getOperand() const;
  virtual unsigned complexity() const;
  virtual bool equalTo(const Formula & f) const;
  virtual void getVars(VariableSet & vars, bool free) const;
};


/* Klasa predstavlja univerzalno kvantifikovanu formulu */
class Forall : public Quantifier {
public:
  using Quantifier::Quantifier;
  virtual Type getType() const;
  virtual void printFormula(ostream & ostr) const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();
  virtual Formula skolem(Signature & s, vector<Variable> && vars);

};


/* Klasa predstavlja egzistencijalnog kvantifikatora */
class Exists : public Quantifier {
public:
  using Quantifier::Quantifier;
  virtual Type getType() const;
  virtual void printFormula(ostream & ostr) const;
  virtual bool eval(const Structure & st, 
		    const Valuation & val) const;
  virtual Formula substitute(const Variable & v, const Term & t);

  virtual Formula simplify();
  virtual Formula nnf();
  virtual Formula pullquants();
  virtual Formula prenex();
  virtual Formula skolem(Signature & s, vector<Variable> && vars);

};

/* Tip podataka kojim se predstavlja domen. U opstem slucaju, domen u logici
   prvog reda moze biti i beskonacan skup. Medjutim, mi cemo se u nasoj
   implementaciji zadrzati na konacnim skupovima, kako bismo mogli da 
   simuliramo izracunavanje interpretacija kvantifikovanih formula. Zato
   ce nam domen uvek biti neki konacan podskup skupa prirodnih brojeva */
typedef std::vector<unsigned> Domain;

/* Apstraktni tip podatka kojim se predstavlja funkcija D^n --> D kojom
   se mogu interpretirati funkcijski simboli arnosti n */
class Function {
private:
  unsigned _arity;
public:
  Function(unsigned arity);
  unsigned getArity();
  virtual unsigned eval(const vector<unsigned> & args = vector<unsigned>()) = 0;
  virtual ~Function() {}
};


/* Apstraktni tip podataka kojim se predstavlja relacija D^n --> {0,1} kojom
   se mogu interpretirati predikatski simboli arnosti n */
class Relation {
private:
  unsigned _arity;
public:
  Relation(unsigned arity);
  unsigned getArity();
  virtual bool eval(const vector<unsigned> & args = vector<unsigned>()) = 0;
  virtual ~Relation() {}
};


/* Klasa koja predstavlja L-strukturu (ili model) nad signaturom L i domenom
   D. Model svakom funkcijskom simbolu pridruzuje funkciju odgovarajuce
   arnosti, dok svakom predikatskom simbolu pridruzuje relaciju odgovarajuce
   arnosti. */
class Structure {
private:
  const Signature & _sig;
  const Domain & _domain;
  map<FunctionSymbol, Function *> _funs;
  map<PredicateSymbol, Relation *> _rels;
  
public:
  Structure(const Signature & sig, const Domain & domain);
  
  const Signature & getSignature() const;
  const Domain & getDomain() const;
  
  /* Dodavanje interpretacije funkcijskom simbolu */
  void addFunction(const FunctionSymbol & fs, Function * f);

  /* Citanje interpretacije datog funkcijskog simbola */
  Function * getFunction(const FunctionSymbol & f) const;

  /* Dodavanje interpretacije predikatskom simbolu */
  void addRelation(const PredicateSymbol & ps, Relation * r);

  /* Citanje interpretacije datog predikatskog simbola */
  Relation * getRelation(const PredicateSymbol & p) const;
 
  ~Structure();
};

/* Klasa kojom se predstavlja valuacija. Valuacijom (nad datim domenom) se
   svakoj varijabli dodeljuje neka vrednost iz domena. Drugim recima, 
   valuacija odredjuje interpretaciju varijabli. */
class Valuation {
private:
  const Domain & _domain;
  map<Variable, unsigned> _values;
public:
  Valuation(const Domain & dom);
  const Domain & getDomain() const;

  /* Postavljanje vrednosti date varijable na datu vrednost */
  void setValue(const Variable & v, unsigned value);

  /* Ocitavanje vrednosti date varijable */
  unsigned getValue(const Variable & v) const;
};

// Definicije funkcija clanica -----------------------------------------

// Funkcije substitucije -----------------------------------------------

Term VariableTerm::substitute(const Variable & v, const Term & t) 
{
  if(_v == v)
    return t;
  else
    return shared_from_this();
}

Term FunctionTerm::substitute(const Variable & v, const Term & t) 
{
  vector<Term> sub_ops;

  for(unsigned i = 0; i < _ops.size(); i++)
    sub_ops.push_back(_ops[i]->substitute(v, t));
  
  return make_shared<FunctionTerm>(_sig, _f, sub_ops);
}

Formula LogicConstant::substitute(const Variable & v, const Term & t) 
{
  return shared_from_this();
}

Formula Atom::substitute(const Variable & v, const Term & t) 
{
  vector<Term> sub_ops;
    
  for(unsigned i = 0; i < _ops.size(); i++)
    sub_ops.push_back(_ops[i]->substitute(v, t));
  
  return make_shared<Atom>(_sig, _p, sub_ops); 
}

Formula Not::substitute(const Variable & v, const Term & t) 
{
  return make_shared<Not>(_op->substitute(v, t));
}

Formula And::substitute(const Variable & v, const Term & t) 
{
  return make_shared<And>(_op1->substitute(v, t), _op2->substitute(v, t));
}

Formula Or::substitute(const Variable & v, const Term & t) 
{
  return make_shared<Or>(_op1->substitute(v, t), _op2->substitute(v, t));
}

Formula Imp::substitute(const Variable & v, const Term & t) 
{
  return make_shared<Imp>(_op1->substitute(v, t), _op2->substitute(v, t));
}

Formula Iff::substitute(const Variable & v, const Term & t) 
{
  return make_shared<Iff>(_op1->substitute(v, t), _op2->substitute(v, t));
}


Formula Forall::substitute(const Variable & v, const Term & t)
{
  if(v == _v)
    return shared_from_this();
    
  /* Ako term sadrzi kvantifikovanu varijablu, tada moramo najpre
     preimenovati kvantifikovanu varijablu (nekom varijablom koja
     nije sadzana ni u termu ni u formuli sto nam daje funkcija
     getUniqueVariable koja je clanica klase Quantifier) */
    if(t->containsVariable(_v))
      {
	Variable new_v = getUniqueVariable(shared_from_this(), t);
	Formula sub_op = _op->substitute(_v, make_shared<VariableTerm>(new_v));
	return make_shared<Forall>(new_v, sub_op->substitute(v, t));
      }
    else
      return make_shared<Forall>(_v, _op->substitute(v, t)); 
}

Formula Exists::substitute(const Variable & v, const Term & t)
{
  if(v == _v)
    return shared_from_this();
  
  /* Ako term sadrzi kvantifikovanu varijablu, tada moramo najpre
     preimenovati kvantifikovanu varijablu (nekom varijablom koja
     nije sadzana ni u termu ni u formuli sto nam daje funkcija
     getUniqueVariable koja je clanica klase Quantifier) */
  if(t->containsVariable(_v))
    {
      Variable new_v = getUniqueVariable(shared_from_this(), t);
      
      Formula sub_op = _op->substitute(_v, make_shared<VariableTerm>(new_v));
      return make_shared<Exists>(new_v, sub_op->substitute(v, t));
    }
  else
    return make_shared<Exists>(_v, _op->substitute(v, t)); 
}

// ---------------------------------------------------------------------

// Funkcije za odredjivanje sintaksne identicnosti termova i formula ---

bool VariableTerm::equalTo(const Term & t) const
{
  return t->getType() == TT_VARIABLE && 
    ((VariableTerm *) t.get())->getVariable() == _v;
}

bool FunctionTerm::equalTo(const Term & t) const
{
  if(t->getType() != TT_FUNCTION)
    return false;
  
  if(_f != ((FunctionTerm *) t.get())->getSymbol())
    return false;
  
  const vector<Term> & t_ops = ((FunctionTerm *) t.get())->getOperands();
  
  if(_ops.size() != t_ops.size())
    return false;
  
  for(unsigned i = 0; i < _ops.size(); i++)
    if(!_ops[i]->equalTo(t_ops[i]))
      return false;
  
  return true;
}

bool LogicConstant::equalTo( const Formula & f) const
{
  return f->getType() == this->getType();
}


bool Atom::equalTo(const Formula & f) const
{
  if(f->getType() != T_ATOM)
    return false;
  
  if(_p != ((Atom *) f.get())->getSymbol())
    return false;
  
  const vector<Term> & f_ops = ((Atom *) f.get())->getOperands();
  
  if(_ops.size() != f_ops.size())
    return false;
  
  for(unsigned i = 0; i < _ops.size(); i++)
    if(!_ops[i]->equalTo(f_ops[i]))
      return false;
  
    return true;
}

bool UnaryConnective::equalTo(const Formula & f) const
{
  return f->getType() == this->getType() && 
    _op->equalTo(((UnaryConnective *)f.get())->getOperand());
}

bool BinaryConnective::equalTo( const Formula & f) const
{
  return f->getType() == this->getType() && 
    _op1->equalTo(((BinaryConnective *)f.get())->getOperand1()) 
    &&  
    _op2->equalTo(((BinaryConnective *)f.get())->getOperand2());
}

bool Quantifier::equalTo(const Formula & f) const
{
  return f->getType() == getType() &&
    ((Quantifier *) f.get())->getVariable() == _v && 
    ((Quantifier *) f.get())->getOperand()->equalTo(_op);
}

// ---------------------------------------------------------------------

// Funkcije za odredjivanje skupa varijabli ----------------------------

void VariableTerm::getVars(VariableSet & vars) const
{
  vars.insert(_v);
}

void FunctionTerm::getVars(VariableSet & vars) const
{
  for(unsigned i = 0; i < _ops.size(); i++)
    _ops[i]->getVars(vars);
}

void LogicConstant::getVars(VariableSet & vars, bool free) const
{
  return;
}

void Atom::getVars(VariableSet & vars, bool free) const
{
  for(unsigned i = 0; i < _ops.size(); i++)
    {
      _ops[i]->getVars(vars);
    }
}

void UnaryConnective::getVars(VariableSet & vars, bool free) const
{
  _op->getVars(vars, free);
}

void BinaryConnective::getVars(VariableSet & vars, bool free) const
{
  _op1->getVars(vars, free);
  _op2->getVars(vars, free);
}

void Quantifier::getVars(VariableSet & vars, bool free) const
{
  bool present = false;

  if(free)
    {
      /* Pamtimo da li je kvantifikovana varijabla vec u skupu slobodnih
	 varijabli */
      if(vars.find(_v) != vars.end())
	present = true;
    }
  
  _op->getVars(vars, free);
  if(!free)
    vars.insert(_v);
  
  if(free)
    {
      /* Ako varijabla ranije nije bila prisutna u skupu slobodnih varijabli,
	 tada je brisemo, zato sto to znaci da se ona pojavljuje samo u 
	 podformuli kvantifikovane formule,a u njoj je vezana kvantifikatorom */
      if(!present && vars.find(_v) != vars.end())
	vars.erase(_v);
    }
}

// ---------------------------------------------------------------------

// Funkcije za odredjivanje slozenosti formule -------------------------

unsigned AtomicFormula::complexity() const
{
  return 0;
}  

unsigned UnaryConnective::complexity() const
{
  return _op->complexity() + 1;
}

unsigned BinaryConnective::complexity() const
{
  return _op1->complexity() + _op2->complexity() + 1;
}

unsigned Quantifier::complexity() const
{
  return _op->complexity() + 1;
}

// ---------------------------------------------------------------------

// Funkcije za stampanje -----------------------------------------------


void VariableTerm::printTerm(ostream & ostr) const
{
  ostr << _v;
}

void FunctionTerm::printTerm(ostream & ostr) const
{
  ostr << _f;

  for(unsigned i = 0; i < _ops.size(); i++)
    {
      if(i == 0)
	ostr << "(";
      ostr << _ops[i];
      if(i != _ops.size() - 1)
	ostr << ",";
      else
	ostr << ")";
    }
}

void True::printFormula(ostream & ostr) const
{
  ostr << "True";
}

void False::printFormula(ostream & ostr) const
{
  ostr << "False";
}

void Atom::printFormula(ostream & ostr) const
{
  ostr << _p;
  for(unsigned i = 0; i < _ops.size(); i++)
    {
      if(i == 0)
	ostr << "(";
      ostr << _ops[i];
      if(i != _ops.size() - 1)
	ostr << ",";
      else
	ostr << ")";
    }
}

void Not::printFormula(ostream & ostr) const
{
  ostr << "(~" << _op << ")";
}

void And::printFormula(ostream & ostr) const
{
  ostr << "(" << _op1 << " /\\ " << _op2 << ")";
}

void Or::printFormula(ostream & ostr) const
{
  ostr << "(" << _op1 << " \\/ " << _op2 << ")";
}

void Imp::printFormula(ostream & ostr) const
{
  ostr << "(" << _op1 << " ==> " << _op2 << ")";
}

void Iff::printFormula(ostream & ostr) const
{
  ostr << "(" << _op1 << " <=> " << _op2 << ")";
}

void Forall::printFormula(ostream & ostr) const
{
  ostr << "(A " << _v << ").(" << _op << ")";
}

void Exists::printFormula(ostream & ostr) const
{
  ostr << "(E " << _v << ").(" << _op << ")";
}


ostream & operator << (ostream & ostr, const Term & t)
{
  t->printTerm(ostr);
  return ostr;
}

ostream & operator << (ostream & ostr, const Formula & f)
{
  f->printFormula(ostr);
  return ostr;
}


// ---------------------------------------------------------------------


// Funkcije za izracunvanje interpretacija -----------------------------

unsigned VariableTerm::eval(const Structure & st, const Valuation & val) const
{
  return val.getValue(_v);
}

unsigned FunctionTerm::eval(const Structure & st, const Valuation & val) const
{
  Function * f = st.getFunction(_f);
  
  vector<unsigned> args;

  for(unsigned i = 0; i < _ops.size(); i++)
    args.push_back(_ops[i]->eval(st, val));

  return f->eval(args);
}  

bool True::eval(const Structure & st, 
		const Valuation & val) const
{
  return true;
}

bool False::eval(const Structure & st, 
		 const Valuation & val) const
{
  return false;
}

bool Atom::eval(const Structure & st, 
		const Valuation & val) const
{
  Relation * r = st.getRelation(_p);
  
  vector<unsigned> args;

  for(unsigned i = 0; i < _ops.size(); i++)
    args.push_back(_ops[i]->eval(st, val));
  
  return r->eval(args);
}

bool Not::eval(const Structure & st, 
	       const Valuation & val) const
{
  return !_op->eval(st, val);
}

bool And::eval(const Structure & st, 
	       const Valuation & val) const
{
  return _op1->eval(st, val) && _op2->eval(st, val);
}


bool Or::eval(const Structure & st, 
	      const Valuation & val) const
{
  return _op1->eval(st, val) || _op2->eval(st, val);
}

bool Imp::eval(const Structure & st, 
		    const Valuation & val) const
{
  return !_op1->eval(st, val) || _op2->eval(st, val);
}

bool Iff::eval(const Structure & st, 
	       const Valuation & val) const
{
  return _op1->eval(st, val) == _op2->eval(st, val);
}

bool Forall::eval(const Structure & st, 
		  const Valuation & val) const
{
  Valuation val_p = val;
  for(unsigned i = 0; i < st.getDomain().size(); i++)
    {
      val_p.setValue(_v, st.getDomain()[i]);
      if(_op->eval(st, val_p) == false)
	return false;
    }
  return true;
}


bool Exists::eval(const Structure & st, 
		  const Valuation & val) const
{
  Valuation val_p = val;
  for(unsigned i = 0; i < st.getDomain().size(); i++)
    {
      val_p.setValue(_v, st.getDomain()[i]);
      if(_op->eval(st, val_p) == true)
	return true;
    }
  return false;
}

// -----------------------------------------------------------------------

// Funkcije za simplifikaciju -------------------------------------------

/* Simplifikacija atomicke formule je trivijalna */
Formula AtomicFormula::simplify()
{
  return shared_from_this();
}

Formula Not::simplify()
{
  Formula simp_op = _op->simplify();
  
  if(simp_op->getType() == T_TRUE)
    return make_shared<False>();
  else if(simp_op->getType() == T_FALSE)
    return make_shared<True>();
  else
    return make_shared<Not>(simp_op);
}

Formula And::simplify()
{
  /* Simplifikacija konjukcije po pravilima A /\ True === A, 
     A /\ False === False i sl. */
  Formula simp_op1 = _op1->simplify();
  Formula simp_op2 = _op2->simplify();
  
  if(simp_op1->getType() == T_TRUE)
    {
      return simp_op2;
    }
  else if(simp_op2->getType() == T_TRUE)
    return simp_op1;
  else if(simp_op1->getType() == T_FALSE ||
	  simp_op2->getType() == T_FALSE)
    return make_shared<False>();
  else
    return make_shared<And>(simp_op1, simp_op2);
}

Formula Or::simplify()
{
  /* Simplifikacija disjunkcije po pravilima: A \/ True === True,
     A \/ False === A, i sl. */
  Formula simp_op1 = _op1->simplify();
  Formula simp_op2 = _op2->simplify();
  
  if(simp_op1->getType() == T_FALSE) 
    return simp_op2;
  else if(simp_op2->getType() == T_FALSE)
    return simp_op1;
  else if(simp_op1->getType() == T_TRUE ||
	  simp_op2->getType() == T_TRUE)
    return make_shared<True>();
  else
    return make_shared<Or>(simp_op1, simp_op2);
}

Formula Imp::simplify()
{
  /* Simplifikacija implikacije po pravilima: A ==> True === True,
     A ==> False === ~A, True ==> A === A, False ==> A === True */
  Formula simp_op1 = _op1->simplify();
  Formula simp_op2 = _op2->simplify();
  
  if(simp_op1->getType() == T_FALSE) 
    return make_shared<True>();
  else if(simp_op2->getType() == T_FALSE)
    return make_shared<Not>(simp_op1);
  else if(simp_op1->getType() == T_TRUE)
    return simp_op2;
  else if(simp_op2->getType() == T_TRUE)
    return make_shared<True>();
  else
    return make_shared<Imp>(simp_op1, simp_op2);
}

Formula Iff::simplify()
{
  /* Ekvivalencija se simplifikuje pomocu pravila:
     True <=> A === A, False <=> A === ~A i sl. */
  
  Formula simp_op1 = _op1->simplify();
  Formula simp_op2 = _op2->simplify();

  if(simp_op1->getType() == T_FALSE && simp_op2->getType() == T_FALSE)
    return make_shared<True>();
  else if(simp_op1->getType() == T_FALSE) 
    return make_shared<Not>(simp_op2);
  else if(simp_op2->getType() == T_FALSE)
    return make_shared<Not>(simp_op1);
  else if(simp_op1->getType() == T_TRUE)
    return simp_op2;
  else if(simp_op2->getType() == T_TRUE)
    return simp_op1;
  else
    return make_shared<Iff>(simp_op1, simp_op2);
}

Formula Forall::simplify()
{
  Formula simp_op = _op->simplify();
  
  /* Ako simplifikovana podformula sadrzi slobodnu varijablu v, tada
     zadrzavamo kvantifikator, u suprotnom ga brisemo */
  if(simp_op->containsVariable(_v, true))
    return  make_shared<Forall>(_v, simp_op);
  else
    return simp_op;
}

Formula Exists::simplify()
{
  Formula simp_op = _op->simplify();

  /* Ako simplifikovana podformula sadrzi slobodnu varijablu v, tada
     zadrzavamo kvantifikator, u suprotnom ga brisemo */
  if(simp_op->containsVariable(_v, true))
    return make_shared<Exists>(_v, simp_op);
  else
    return simp_op;
}



// ---------------------------------------------------------------------


// NNF funkcije --------------------------------------------------------

Formula AtomicFormula::nnf()
{
  return shared_from_this();
}

Formula Not::nnf()
{
  /* Eliminacija dvojne negacije */
  if(_op->getType() == T_NOT)
    {
      Not * not_op = (Not *) _op.get();
      return not_op->getOperand()->nnf();
    }
  /* De-Morganov zakon ~(A/\B) === ~A \/ ~B, pa zatim rekurzivna 
     primena nnf-a na ~A i ~B */
  else if(_op->getType() == T_AND)
    {
      And * and_op =  (And *) _op.get();
	
      return make_shared<Or>(make_shared<Not>(and_op->getOperand1())->nnf(),
			     make_shared<Not>(and_op->getOperand2())->nnf());
      
    }
  /* De-Morganov zakon ~(A\/B) === ~A /\ ~B, pa zatim rekurzivna 
     primena nnf-a na ~A i ~B */
  else if(_op->getType() == T_OR)
    {
      Or * or_op =  (Or *) _op.get();
      
      return make_shared<And>(make_shared<Not>(or_op->getOperand1())->nnf(),
			      make_shared<Not>(or_op->getOperand2())->nnf());
      
    }
  /* De-Morganov zakon ~(A==>B) === A /\ ~B, pa zatim rekurzivna
     primena nnf-a na A i ~B */
  else if(_op->getType() == T_IMP)
    {
      Imp * imp_op =  (Imp *) _op.get();
	
      return make_shared<And>(imp_op->getOperand1()->nnf(),
			      make_shared<Not>(imp_op->getOperand2())->nnf());
      
    }
  /* Primena pravila ~(A<=>B) === (A /\ ~B) \/ (B /\ ~A) */
  else if(_op->getType() == T_IFF)
    {
      Iff * iff_op =  (Iff *) _op.get();
      
      return make_shared<Or>(make_shared<And>(iff_op->getOperand1()->nnf(),
					      make_shared<Not>(iff_op->getOperand2())->nnf()),
			     make_shared<And>(iff_op->getOperand2()->nnf(),
					      make_shared<Not>(iff_op->getOperand1())->nnf()));
      
    }
  /* Primena pravila ~(forall x) A === (exists x) ~A */ 
  else if(_op->getType() == T_FORALL)
    {
      Forall * forall_op = (Forall *) _op.get();      

      return make_shared<Exists>(forall_op->getVariable(), 
				 make_shared<Not>(forall_op->getOperand())->nnf());
    }
  /* Primena pravila ~(exists x) A === (forall x) ~A */
  else if(_op->getType() == T_EXISTS)
    {
      Exists * exists_op = (Exists *) _op.get();      
      
      return make_shared<Forall>(exists_op->getVariable(), 
				 make_shared<Not>(exists_op->getOperand())->nnf());
    }
  else
    {
      return shared_from_this();
    } 
}

Formula And::nnf()
{
  return make_shared<And>(_op1->nnf(), _op2->nnf());
}

Formula Or::nnf()
{
  return make_shared<Or>(_op1->nnf(), _op2->nnf());
}

Formula Imp::nnf()
{
  /* Eliminacija implikacije, pa zatim rekurzivna primena nnf()-a */
  return make_shared<Or>(make_shared<Not>(_op1)->nnf(), _op2->nnf());
}

Formula Iff::nnf()
{
  /* Eliminacija ekvivalencije, pa zatim rekurzivna primena nnf()-a.
     Primetimo da se ovde velicina formule duplira */
  return make_shared<And>(make_shared<Or>(make_shared<Not>(_op1)->nnf(), _op2->nnf()),
			  make_shared<Or>(make_shared<Not>(_op2)->nnf(), _op1->nnf()));
}

Formula Forall::nnf()
{
  return make_shared<Forall>(_v, _op->nnf());
}

Formula Exists::nnf()
{
  return make_shared<Exists>(_v, _op->nnf());
}

// Funkcije za izvlacenje kvantifikatora  ------------------------------

Formula AtomicFormula::pullquants()
{
  /* U slucaju atomicke formule, ne radimo nista */
  return shared_from_this();
}

Formula Not::pullquants()
{
  /* U slucaju negacije, ne radimo nista, s obzirom da je formula
     vec u NNF-u, pa su negacije spustene do nivoa atoma. */
  return shared_from_this();
}

Formula And::pullquants()
{
  /* Ako je formula oblika (forall x) A /\ (forall y) B */
  if(_op1->getType() == T_FORALL && _op2->getType() == T_FORALL)
    {
      Forall * fop1 = (Forall *) _op1.get();
      Forall * fop2 = (Forall *) _op2.get();
      
      /* Specijalno, ako je formula oblika (forall x) A /\ (forall x) B
	 tada ne moramo da preimenujemo vezanu promenljivu, vec samo
	 izvlacimo kvantifikator (forall x) (A /\ B), a zatim iz podformule
	 rekurzivno izvucemo kvantifikatore */
      if(fop1->getVariable() == fop2->getVariable())
	{
	  return make_shared<Forall>(fop1->getVariable(), 
				     make_shared<And>(fop1->getOperand(),
						      fop2->getOperand())->pullquants());
	}
      /* U suprotnom, uvodimo novu promenljivu koja se ne pojavljuje
	 ni u jednoj od  formula A i B */
      else {
	Variable var = 
	  getUniqueVariable(fop1->getOperand(), fop2->getOperand());
	
	return make_shared<Forall>(var, 
				   make_shared<And>(fop1->
						    getOperand()->
						    substitute(fop1->getVariable(),
							       make_shared<VariableTerm>(var)),
						    fop2->
						    getOperand()->
						    substitute(fop2->getVariable(),
							       make_shared<VariableTerm>(var)))->pullquants());
      } 
    }
  /* Slucaj ((exists x) A) /\ B */
  else if(_op1->getType() == T_EXISTS)
    {
      Exists * eop1 = (Exists *) _op1.get();
      
      /* Ako x ne postoji kao slobodna varijabla u B, tada je dovoljno
	 samo izvuci kvantifikator (exists x) (A /\ B) */
      if(!_op2->containsVariable(eop1->getVariable(), true))
	{
	  return make_shared<Exists>(eop1->getVariable(), 
				     make_shared<And>(eop1->getOperand(), _op2)->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(eop1->getOperand(), _op2);
	  return make_shared<Exists>(var, 
				     make_shared<And>(eop1->getOperand()->
						      substitute(eop1->getVariable(), 
								 make_shared<VariableTerm>(var)),
						      _op2)->pullquants());
	}
    }
  /* Slucaj A /\ (exists x) B */
  else if(_op2->getType() == T_EXISTS)
    {
      Exists * eop2 = (Exists *) _op2.get();
      
      /* Ako x ne postoji kao slobodna varijabla u A, tada je dovoljno
	 samo izvuci kvantifikator (exists x) (A /\ B) */
      if(!_op1->containsVariable(eop2->getVariable(), true))
	{
	  return make_shared<Exists>(eop2->getVariable(), 
				     make_shared<And>(_op1, eop2->getOperand())->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(eop2->getOperand(), _op1);
	  return make_shared<Exists>(var, 
				     make_shared<And>(_op1, 
						      eop2->getOperand()->
						      substitute(eop2->getVariable(), 
								 make_shared<VariableTerm>(var)))->pullquants());
	}
    }
  /* Slucaj ((forall x) A) /\ B */
  else if(_op1->getType() == T_FORALL)
    {
      Forall * fop1 = (Forall *) _op1.get();
      
      /* Ako x ne postoji kao slobodna varijabla u B, tada je dovoljno
	 samo izvuci kvantifikator (forall x) (A /\ B) */
      if(!_op2->containsVariable(fop1->getVariable(), true))
	{
	  return make_shared<Forall>(fop1->getVariable(), 
				     make_shared<And>(fop1->getOperand(), _op2)->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(fop1->getOperand(), _op2);
	  return make_shared<Forall>(var, make_shared<And>(fop1->getOperand()->
							   substitute(fop1->getVariable(), 
								      make_shared<VariableTerm>(var)),
							   _op2)->pullquants());
	}
    }
  /* Slucaj A /\ (forall x) B */
  else if(_op2->getType() == T_FORALL)
    {
      Forall * fop2 = (Forall *) _op2.get();
      
      /* Ako x ne postoji kao slobodna varijabla u A, tada je dovoljno
	 samo izvuci kvantifikator (forall x) (A /\ B) */
      if(!_op1->containsVariable(fop2->getVariable(), true))
	{
	  return make_shared<Forall>(fop2->getVariable(), 
				     make_shared<And>(_op1, fop2->getOperand())->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(fop2->getOperand(), _op1);
	  return make_shared<Forall>(var, 
				     make_shared<And>(_op1, 
						      fop2->getOperand()->
						      substitute(fop2->getVariable(), 
								 make_shared<VariableTerm>(var)))->pullquants());
	}
    }
  /* Formula je oblika A /\ B, gde ni A ni B nemaju kvantifikator kao vodeci
     veznik. U tom slucaju, ne radimo nista, jer su svi kvantifikatori izvuceni. */
  else
    {
      return shared_from_this();
    }
}



Formula Or::pullquants()
{
  /* Ako je formula oblika (exists x) A \/ (exists y) B */
  if(_op1->getType() == T_EXISTS && _op2->getType() == T_EXISTS)
    {
      Exists * fop1 = (Exists *) _op1.get();
      Exists * fop2 = (Exists *) _op2.get();
      
      /* Specijalno, ako je formula oblika (exists x) A \/ (exists x) B
	 tada ne moramo da preimenujemo vezanu promenljivu, vec samo
	 izvlacimo kvantifikator (exists x) (A \/ B), a zatim iz podformule
	 rekurzivno izvucemo kvantifikatore */
      if(fop1->getVariable() == fop2->getVariable())
	{
	  return make_shared<Exists>(fop1->getVariable(), 
				     make_shared<Or>(fop1->getOperand(),
						      fop2->getOperand())->pullquants());
	}
      /* U suprotnom, uvodimo novu promenljivu koja se ne pojavljuje
	 ni u jednoj od  formula A i B */
      else {
	Variable var = 
	  getUniqueVariable(fop1->getOperand(), fop2->getOperand());
	
	return make_shared<Exists>(var, 
				   make_shared<Or>(fop1->
						   getOperand()->
						   substitute(fop1->getVariable(),
							      make_shared<VariableTerm>(var)),
						   fop2->
						   getOperand()->
						   substitute(fop2->getVariable(),
							      make_shared<VariableTerm>(var)))->pullquants());
      } 
    }
  /* Slucaj ((exists x) A) \/ B */
  else if(_op1->getType() == T_EXISTS)
    {
      Exists * eop1 = (Exists *) _op1.get();
      
      /* Ako x ne postoji kao slobodna varijabla u B, tada je dovoljno
	 samo izvuci kvantifikator (exists x) (A /\ B) */
      if(!_op2->containsVariable(eop1->getVariable(), true))
	{
	  return make_shared<Exists>(eop1->getVariable(), 
				     make_shared<Or>(eop1->getOperand(), _op2)->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(eop1->getOperand(), _op2);
	  return make_shared<Exists>(var, 
				     make_shared<Or>(eop1->getOperand()->
						     substitute(eop1->getVariable(), 
								make_shared<VariableTerm>(var)),
						     _op2)->pullquants());
	}
    }
  /* Slucaj A \/ (exists x) B */
  else if(_op2->getType() == T_EXISTS)
    {
      Exists * eop2 = (Exists *) _op2.get();
      
      /* Ako x ne postoji kao slobodna varijabla u A, tada je dovoljno
	 samo izvuci kvantifikator (exists x) (A \/ B) */
      if(!_op1->containsVariable(eop2->getVariable(), true))
	{
	  return make_shared<Exists>(eop2->getVariable(), 
				     make_shared<Or>(_op1, eop2->getOperand())->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(eop2->getOperand(), _op1);
	  return make_shared<Exists>(var, 
				     make_shared<Or>(_op1, 
						     eop2->getOperand()->
						     substitute(eop2->getVariable(), 
								make_shared<VariableTerm>(var)))->pullquants());
	}
    }
  /* Slucaj ((forall x) A) \/ B */
  else if(_op1->getType() == T_FORALL)
    {
      Forall * fop1 = (Forall *) _op1.get();
      
      /* Ako x ne postoji kao slobodna varijabla u B, tada je dovoljno
	 samo izvuci kvantifikator (forall x) (A \/ B) */
      if(!_op2->containsVariable(fop1->getVariable(), true))
	{
	  return make_shared<Forall>(fop1->getVariable(), 
				     make_shared<Or>(fop1->getOperand(), _op2)->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(fop1->getOperand(), _op2);
	  return make_shared<Forall>(var, make_shared<Or>(fop1->getOperand()->
							  substitute(fop1->getVariable(), 
								     make_shared<VariableTerm>(var)),
							  _op2)->pullquants());
	}
    }
  /* Slucaj A \/ (forall x) B */
  else if(_op2->getType() == T_FORALL)
    {
      Forall * fop2 = (Forall *) _op2.get();
      
      /* Ako x ne postoji kao slobodna varijabla u A, tada je dovoljno
	 samo izvuci kvantifikator (forall x) (A \/ B) */
      if(!_op1->containsVariable(fop2->getVariable(), true))
	{
	  return make_shared<Forall>(fop2->getVariable(), 
				     make_shared<Or>(_op1, fop2->getOperand())->pullquants());
	}
      /* u suprotnom, moramo da preimenujemo vezanu varijablu */
      else
	{
	  Variable var = getUniqueVariable(fop2->getOperand(), _op1);
	  return make_shared<Forall>(var, 
				     make_shared<Or>(_op1, 
						     fop2->getOperand()->
						     substitute(fop2->getVariable(), 
								make_shared<VariableTerm>(var)))->pullquants());
	}
    }
  /* Formula je oblika A \/ B, gde ni A ni B nemaju kvantifikator kao vodeci
     veznik. U tom slucaju, ne radimo nista, jer su svi kvantifikatori izvuceni. */
  else
    {
      return shared_from_this();
    }

}

Formula Imp::pullquants()
{
  /* Implikacija ne bi trebalo da se pojavi, jer je formula vec u NNF-u */
  throw "pullquants not applicable";
}

Formula Iff::pullquants()
{
  /* Ekvivalencija ne bi trebalo da se pojavi, jer je formula vec u NNF-u */
  throw "pullquants not applicable";
}

Formula Forall::pullquants()
{
  /* U slucaju Forall formule, ne radimo nista */
  return shared_from_this();
}

Formula Exists::pullquants()
{
  /* U slucaju Exists formule, ne radimo nista */
  return shared_from_this();
}


// ---------------------------------------------------------------------


// ---------------------------------------------------------------------

// Funkcije za PRENEX normalnu formu 

Formula AtomicFormula::prenex()
{
  /* U slucaju atomicke formule ne radimo nista */
  return shared_from_this(); 
}

Formula Not::prenex()
{
  /* U slucaju negacije ne radimo nista, zato sto podrazumevamo
     da je formula vec u NNF-u, pa su negacije spustene do atoma */
  return shared_from_this();
}

Formula And::prenex()
{
  /* U slucaju formule oblika A /\ B, najpre transforimisemo A i B 
     u PRENEX, a zatim iz dobijene konjukcije izvucemo kvantifikatore */
  
  Formula pr_op1 = _op1->prenex();
  Formula pr_op2 = _op2->prenex();
  
  return make_shared<And>(pr_op1, pr_op2)->pullquants();
}

Formula Or::prenex()
{
  /* U slucaju formule oblika A \/ B, najpre transforimisemo A i B 
     u PRENEX, a zatim iz dobijene disjunkcije izvucemo kvantifikatore */

  Formula pr_op1 = _op1->prenex();
  Formula pr_op2 = _op2->prenex();
  
  return make_shared<Or>(pr_op1, pr_op2)->pullquants();
}

Formula Imp::prenex()
{
  /* Implikacija ne bi trebalo da se pojavi, jer je formula vec u NNF-u */
  throw "Prenex not applicable";
}

Formula Iff::prenex()
{
  /* Ekvivalencija ne bi trebalo da se pojavi, jer je formula vec u NNF-u */
  throw "Prenex not applicable";
}

Formula Forall::prenex()
{
  /* U slucaju univerzalnog kvantifikatora, potrebno je samo podformulu
     svesti na prenex formu */
  return make_shared<Forall>(_v, _op->prenex());
}

Formula Exists::prenex()
{
  /* U slucaju egzistencijalnog kvantifikatora, potrebno je samo podformulu
     svesti na prenex formu */
  return make_shared<Exists>(_v, _op->prenex());
}

// -----------------------------------------------------------------------

// Funkcije za skolemizaciju ---------------------------------------------

/* Kod funkcija za skolemizaciju, podrazumeva se da je formula u PRENEX
   normalnoj formi, tj. oblika (Q1 x1) (Q2 x2) ... (Qn xn) A, gde su 
   Qi \in {forall, exists}, dok je A formula bez kvantifikatora koja ne
   sadrzi druge varijable osim x1,...,xn */

Formula BaseFormula::skolem(Signature & s, vector<Variable> && vars)
{
  /* Podrazumevano, za formulu bez kvantifikatora, ne radimo nista */
  return shared_from_this();
}

Formula Forall::skolem(Signature & s, vector<Variable> && vars)
{
  /* Ako je formula oblika (forall x) A, tada samo dodajemo varijablu
     x u sekvencu univerzalno kvantifikovanih varijabli, i zatim pozivamo
     rekurzivni poziv za podformulu */
  vars.push_back(_v);
  return make_shared<Forall>(_v, _op->skolem(s, std::move(vars)));
}

Formula Exists::skolem(Signature & s, vector<Variable> && vars)
{
  /* Kada naidjemo na egzistencijalni kvantifikator (exists y), 
     tada je potrebno eliminisati ga, uvodjenjem novog funkcijskog simbola 
     u signaturu, pa zato najpre uvodimo novi simbol koji nije prisutan u 
     signaturu s */
  FunctionSymbol f = getUniqueFunctionSymbol(s);
  
  /* Dodajemo f u signaturu sa arnoscu k = vars.size(), tj. arnost je jednaka
     broju univerzalno kvantifikovanih varijabli koje prethode egzistencijalnom
     kvantifikatoru. Primetimo da ako je k = 0, tj. ako nije bilo univerzalnih
     kvantifikatora pre datog egzistencijalnog, tada ce se uvesti u signaturu
     novi funkcijski simbol arnosti 0, tj. simbol konstante 
     (Skolemova konstanta). */
  s.addFunctionSymbol(f, vars.size());

  /* Kreiramo vektor termova x1,x2,...,xk */
  vector<Term> varTerms;

  for(unsigned i = 0; i < vars.size(); i++)
    varTerms.push_back(make_shared<VariableTerm>(vars[i]));

  /* Kreiramo term f(x1,...,xk) */
  Term t = make_shared<FunctionTerm>(s, f, varTerms);

  /* Zamenjujemo u podformuli y -> f(x1,...,xk), a zatim nastavljamo
     rekurzivno skolemizaciju u podformuli. */
  return _op->substitute(_v, t)->skolem(s, std::move(vars));
}

// -----------------------------------------------------------------------

// Ostale funkcije clanice -----------------------------------------------

// Klasa Signature -------------------------------------------------------

void Signature::addFunctionSymbol(const FunctionSymbol & f, unsigned arity)
{
  _functions.insert(make_pair(f, arity));
}

void Signature::addPredicateSymbol(const PredicateSymbol & p, unsigned arity)
{
  _predicates.insert(make_pair(p, arity));
}

bool Signature::checkFunctionSymbol(const FunctionSymbol & f, 
				    unsigned & arity) const
{
  map<FunctionSymbol, unsigned>::const_iterator it = _functions.find(f);
  
  if(it != _functions.end())
    {
      arity = it->second;
      return true;
    }
  else
    return false;
}

bool Signature::checkPredicateSymbol(const PredicateSymbol & f, 
				     unsigned & arity) const
{
  map<PredicateSymbol, unsigned>::const_iterator it = _predicates.find(f);
  
  if(it != _predicates.end())
    {
      arity = it->second;
      return true;
    }
  else
    return false;
}

// -----------------------------------------------------------------------

// Klasa Function --------------------------------------------------------

Function::Function(unsigned arity)
  :_arity(arity)
{}

unsigned Function::getArity()
{
  return _arity;
}
// ----------------------------------------------------------------------

// Klasa Relation -------------------------------------------------------

Relation::Relation(unsigned arity)
  :_arity(arity)
{}

unsigned Relation::getArity()
{
  return _arity;
}
// ----------------------------------------------------------------------

// Klasa Structure ------------------------------------------------------

Structure::Structure(const Signature & sig, const Domain & domain)
  :_sig(sig),
   _domain(domain)
{}

const Signature & Structure::getSignature() const
{
  return _sig;
}

const Domain & Structure::getDomain() const
{
  return _domain;
}

void Structure::addFunction(const FunctionSymbol & fs, Function * f)
{
  unsigned arity;
  if(!_sig.checkFunctionSymbol(fs, arity) || arity != f->getArity())
    throw "Function arity mismatch";
  
  _funs.insert(make_pair(fs, f));
}

Function * Structure::getFunction(const FunctionSymbol & f) const
{
  map<FunctionSymbol, Function *>::const_iterator it = 
    _funs.find(f);
  
  if(it != _funs.end())
    {
      return it->second;
    }
  else
    throw "Function symbol unknown!";
}

void Structure::addRelation(const PredicateSymbol & ps, Relation * r)
{
  unsigned arity;
  if(!_sig.checkPredicateSymbol(ps, arity) || arity != r->getArity())
    throw "Relation arity mismatch";
  
  _rels.insert(make_pair(ps, r));
}

Relation * Structure::getRelation(const PredicateSymbol & p) const
{
  map<PredicateSymbol, Relation *>::const_iterator it = 
    _rels.find(p);
  
  if(it != _rels.end())
    {
      return it->second;
    }
  else
    throw "Predicate symbol unknown!";
}

Structure::~Structure()
{
  for(const auto & p : _funs)
    delete p.second;
  for(const auto & p : _rels)
    delete p.second;
}

// ----------------------------------------------------------------------

// Klasa Valuation ------------------------------------------------------

Valuation::Valuation(const Domain & dom)
    :_domain(dom)
{}
  
const Domain & Valuation::getDomain() const
{
  return _domain;
}

void Valuation::setValue(const Variable & v, unsigned value)
{
  if(find(_domain.begin(), _domain.end(), value) ==
     _domain.end())
    throw "Value not in domain!";
  
  _values[v] = value;
}

unsigned Valuation::getValue(const Variable & v) const
{
  map<Variable, unsigned>::const_iterator it = 
    _values.find(v);
  
  if(it != _values.end())
    return it->second;
  else
    throw "Variable unknown!";
}

// -----------------------------------------------------------------------

// Klasa BaseTerm ------------------------------------------------------------

bool BaseTerm::containsVariable(const Variable & v) const
{
  VariableSet vars;
  getVars(vars);
  return vars.find(v) != vars.end();
}
// -----------------------------------------------------------------------

// Klasa VariableTerm ----------------------------------------------------

VariableTerm::VariableTerm(const Variable & v)
  :_v(v)
{}

BaseTerm::Type VariableTerm::getType() const
{
  return TT_VARIABLE;
}

const Variable & VariableTerm::getVariable() const
{
  return _v;
}

// -----------------------------------------------------------------------

// Klasa FunctionTerm ----------------------------------------------------

FunctionTerm::FunctionTerm(const Signature & s, const FunctionSymbol & f, 
			   const vector<Term> & ops)
  :_sig(s),
   _f(f),
   _ops(ops)
{
  unsigned arity;
  if(!_sig.checkFunctionSymbol(_f, arity) || arity != _ops.size())
    throw "Syntax error!";
}

FunctionTerm::FunctionTerm(const Signature & s, const FunctionSymbol & f, 
			   vector<Term> && ops)
  :_sig(s),
   _f(f),
   _ops(std::move(ops))
{
  unsigned arity;
  if(!_sig.checkFunctionSymbol(_f, arity) || arity != _ops.size())
    throw "Syntax error!";
}


BaseTerm::Type FunctionTerm::getType() const
{
  return TT_FUNCTION;
}


const Signature & FunctionTerm::getSignature() const
{
  return _sig;
}

const FunctionSymbol & FunctionTerm::getSymbol() const
{
  return _f;
}

const vector<Term> & FunctionTerm::getOperands() const
{
  return _ops;
}

// ----------------------------------------------------------------------

// Klasa BaseFormula --------------------------------------------------------

bool BaseFormula::containsVariable(const Variable & v, bool free) const
{
  VariableSet vars;
  getVars(vars, free);
  return vars.find(v) != vars.end();
}

// ----------------------------------------------------------------------

template <typename T1, typename T2>
Variable getUniqueVariable(const T1 & e1, const T2 & e2)
{
  static unsigned i = 0;
  
  Variable v;
  
  do {    
    v = string("uv") + to_string(++i);
  } while(e1->containsVariable(v) || e2->containsVariable(v));
  
  return v;
}

FunctionSymbol getUniqueFunctionSymbol(const Signature & s)
{
  static unsigned i = 0;
  unsigned arity;

  FunctionSymbol f;
  
  do {
    f = string("uf") + to_string(++i);
  } while(s.checkFunctionSymbol(f, arity));
  
  return f;
}

// ----------------------------------------------------------------------

// Klasa AtomicFormula --------------------------------------------------

// Klasa LogicConstant --------------------------------------------------

BaseFormula::Type True::getType() const
{
  return T_TRUE;
}
// ----------------------------------------------------------------------

// Klasa False ----------------------------------------------------------

BaseFormula::Type False::getType() const
{
  return T_FALSE;
}

// ----------------------------------------------------------------------

// Klasa Atom -----------------------------------------------------------

Atom::Atom(const Signature & s, 
	   const PredicateSymbol & p, 
	   const vector<Term> & ops)
  :_sig(s),
   _p(p),
   _ops(ops)
{
  unsigned arity;
  if(!_sig.checkPredicateSymbol(_p, arity) || arity != _ops.size())
    throw "Syntax error!";
}

Atom::Atom(const Signature & s, 
	   const PredicateSymbol & p, 
	   vector<Term> && ops)
  :_sig(s),
   _p(p),
   _ops(std::move(ops))
{
  unsigned arity;
  if(!_sig.checkPredicateSymbol(_p, arity) || arity != _ops.size())
    throw "Syntax error!";
}

const PredicateSymbol & Atom::getSymbol() const
{
  return _p;
}

const Signature & Atom::getSignature() const
{
  return _sig;
}

const vector<Term> & Atom::getOperands() const
{
  return _ops;
}


BaseFormula::Type Atom::getType() const
{
  return T_ATOM;
}

// -----------------------------------------------------------------------

// Klasa UnaryConnective -------------------------------------------------

UnaryConnective::UnaryConnective(const Formula & op)
  :_op(op)
{}

const Formula & UnaryConnective::getOperand() const
{
  return _op;
}

// -----------------------------------------------------------------------

// Klasa Not -------------------------------------------------------------

BaseFormula::Type Not::getType() const
{
  return T_NOT;
}

// -----------------------------------------------------------------------

// Klasa BinaryConnective ------------------------------------------------

BinaryConnective::BinaryConnective( const Formula & op1,  const Formula & op2)
  :_op1(op1),
   _op2(op2)
{}

const Formula & BinaryConnective::getOperand1() const
{
  return _op1;
  }

const Formula & BinaryConnective::getOperand2() const
{
  return _op2;
}
  

// Klasa And ---------------------------------------------------------------

BaseFormula::Type And::getType() const
{
  return T_AND;
}

// -------------------------------------------------------------------------

// Klasa Or ----------------------------------------------------------------

BaseFormula::Type Or::getType() const
{
  return T_OR;
}

// -------------------------------------------------------------------------

// Klasa Imp ---------------------------------------------------------------

BaseFormula::Type Imp::getType() const
{
  return T_IMP;
}

// -------------------------------------------------------------------------

// Klasa Iff ---------------------------------------------------------------

BaseFormula::Type Iff::getType() const
{
  return T_IFF;
}

// -------------------------------------------------------------------------
  
// Klasa Quantifier --------------------------------------------------------

Quantifier::Quantifier(const Variable & v, const Formula & op)
  :_v(v),
   _op(op)
{}

const Variable & Quantifier::getVariable() const
{
  return _v;
}

const Formula & Quantifier::getOperand() const
{
  return _op;
}

// ------------------------------------------------------------------------

// Klasa Forall -----------------------------------------------------------

BaseFormula::Type Forall::getType() const
{
  return T_FORALL;
}

// ------------------------------------------------------------------------

// Klasa Exists -----------------------------------------------------------

BaseFormula::Type Exists::getType() const
{
  return T_EXISTS;
}

// -----------------------------------------------------------------------

/* Uopstena supstitucija */
typedef vector< pair<Variable, Term> > Substitution;

/* Funkcija prikazuje uopstenu supstituciju u preglednom obliku */
ostream & operator << (ostream & ostr, const Substitution & sub);

ostream & operator << (ostream & ostr, const Substitution & sub)
{
  ostr << "[ ";
  for(unsigned i = 0; i < sub.size(); i++)
    {     
      ostr << sub[i].first << " ---> " << sub[i].second;
      if(i < sub.size() - 1)
	ostr << ", ";      
    }
  ostr << " ]";

  return ostr;
}

/* Funkcije za unifikaciju */

typedef list< pair<Term, Term> > TermPairs;

/* Prikazuje na izlazu skup parova termova koje zelimo 
   da unifikujemo */
ostream & operator << (ostream & ostr, const TermPairs & pairs);

ostream & operator << (ostream & ostr, const TermPairs & pairs)
{
  for(const auto & p : pairs)
    {
      ostr << "(" << p.first << ", " << p.second << ") ";      
    }
  return ostr;
}

/* Ispituje da li je skup parova termova unifikabilan, i vraca
   najopstiji unifikator ako jeste */
bool unify(const TermPairs & pairs, Substitution & sub);

/* Pomocna funkcije za unifikaciju -- transformise skup parova
   termova primenjujuci pravila iz algoritma. */
bool do_unify(TermPairs & pairs);

/* Primenjuje pravilo factoring */
void applyFactoring(TermPairs & pairs);

/* Primenjuje pravilo tautology */
void applyTautology(TermPairs & pairs);

/* Primenjuje pravilo orientation i vraca true ako je tim pravilom nesto
   promenjeno */
bool applyOrientation(TermPairs & pairs);

/* Primenjuje pravilo decomposition/collision i vraca true ako je tim
   pravilom nesto promenjeno */
bool applyDecomposition(TermPairs & pairs, bool & collision);

/* Primenjuje pravilo application/cycle i vraca true ako je tim pravilom 
   nesto promenjeno. */
bool applyApplication(TermPairs & pairs, bool & cycle);

bool unify(const TermPairs & pairs, Substitution & sub)
{
  TermPairs res_pairs = pairs;

  if(!do_unify(res_pairs))
    return false;

  for(auto i = res_pairs.cbegin(); i != res_pairs.cend(); i++)
    {
      VariableTerm * vt = (VariableTerm *) (*i).first.get();
      sub.push_back(make_pair(vt->getVariable(), (*i).second));
    }
  
  return true;
}

bool do_unify(TermPairs & pairs)
{
  bool repeat =  false;
  bool collision = false;
  bool cycle = false;

  do {
    
    applyFactoring(pairs);    
    applyTautology(pairs);

    repeat = 
      applyOrientation(pairs) ||
      applyDecomposition(pairs, collision) ||
      applyApplication(pairs, cycle);
    
    if(collision || cycle)
      return false;
    
  } while(repeat);
  
  return true;
}


void applyFactoring(TermPairs & pairs)
{
  for(auto i = pairs.begin(); i != pairs.end(); i++)
    {
      auto j = i;
      j++;
      while(j != pairs.end())
	{
	  if((*j).first->equalTo((*i).first) &&
	     (*j).second->equalTo((*i).second))
	    {
	      cout << "Factoring applied: (" << (*j).first << "," << (*j).second << ")" << endl;
	      auto erase_it = j;
	      j++;
	      pairs.erase(erase_it);
	      cout << pairs << endl;
	    }
	  else
	    j++;
	}
    }
}

void applyTautology(TermPairs & pairs)
{
  auto i = pairs.begin();
  
  while(i != pairs.end())
    {
      if((*i).first->equalTo((*i).second))
	{
	  cout << "Tautology applied: (" << (*i).first << "," << (*i).second << ")" << endl;
	  auto erase_it = i;
	  i++;
	  pairs.erase(erase_it);
	  cout << pairs << endl;
	}
      else
	i++;
    }
}

bool applyOrientation(TermPairs & pairs)
{
  bool ret = false;

  for(auto i = pairs.begin(); i != pairs.end(); i++)
    {
      if((*i).first->getType() != BaseTerm::TT_VARIABLE &&
	 (*i).second->getType() == BaseTerm::TT_VARIABLE)
	{
	  cout << "Orientation applied: (" << (*i).first << "," << (*i).second << ")" << endl;
	  swap((*i).first, (*i).second);
	  ret = true;
	  cout << pairs << endl;
	}
    }

  return ret;
}

bool applyDecomposition(TermPairs & pairs, bool & collision)
{
  bool ret = false;
  
  auto i = pairs.begin();
  while(i != pairs.end())
    {
      if((*i).first->getType() == BaseTerm::TT_FUNCTION &&
	 (*i).second->getType() == BaseTerm::TT_FUNCTION)
	{
	  FunctionTerm * ff = (FunctionTerm *) (*i).first.get();
	  FunctionTerm * fs = (FunctionTerm *) (*i).second.get();

	  if(ff->getSymbol() == fs->getSymbol())
	    {
	      const vector<Term> & ff_ops = ff->getOperands();
	      const vector<Term> & fs_ops = fs->getOperands();

	      for(unsigned k = 0; k < ff_ops.size(); k++)
		{
		  pairs.push_back(make_pair(ff_ops[k], fs_ops[k]));
		}

	      cout << "Decomposition applied: (" << (*i).first << "," << (*i).second << ")" << endl;
	      auto erase_it = i;
	      i++;
	      pairs.erase(erase_it);
	      cout << pairs << endl;
	      ret = true;
	    }
	  else
	    {
	      cout << "Collision detected: " << ff->getSymbol() << " != " << fs->getSymbol() << endl;
	      collision = true;
	      return true;
	    }
	  
	}
      else
	i++;
    }

  collision = false;
  return ret;

}

bool applyApplication(TermPairs & pairs, bool & cycle)
{
  bool ret = false;

  for(auto i = pairs.begin(); i != pairs.end(); i++)
    {
      if((*i).first->getType() == BaseTerm::TT_VARIABLE)
	{
	  VariableTerm * vt = (VariableTerm *) (*i).first.get();
	  if((*i).second->containsVariable(vt->getVariable()))
	    {
	      cycle = true;
	      cout << "Cycle detected: " << (*i).second <<  " contains " << vt->getVariable() << endl;
	      return true;
	    }
	  else
	    {
	      bool changed = false;
	      for(auto j = pairs.begin(); j != pairs.end(); j++)
		{
		  if(j != i)
		    {
		      if((*j).first->containsVariable(vt->getVariable()))
			{
			  (*j).first = (*j).first->
			    substitute(vt->getVariable(),
				       (*i).second);
			  ret = true;
			  changed = true;
			}
		      if((*j).second->containsVariable(vt->getVariable()))
			{
			  (*j).second = (*j).second->
			    substitute(vt->getVariable(),
				       (*i).second);
			  ret = true;
			  changed = true;
			}
		    }
		}
	      if(changed)
		{
		  cout << "Application applied: (" << (*i).first << "," << (*i).second << ")" << endl; 
		  cout << pairs << endl;
		}
	    }
	}
    }
  cycle = false;
  return ret;
}


// -----------------------------------------------------------------------

typedef struct criticalPair
{
    Term t1;
    Term t2;
}CriticalPair;

typedef set<CriticalPair> CriticalPairs;
typedef set<Formula> RewriteSystem;


Term getSubterm (const Term &t){
	FunctionTerm *ft  = (FunctionTerm*) t.get();
	vector<Term> ops1 = ft->getOperands();

	vector<Term>::iterator l1_prim = ops1.begin();
	for (;l1_prim!=ops1.end(); l1_prim++)
	{
		//TODO dodati proveru da li je function term
		return *l1_prim;
	}

}

void getAllCriticalPairs (CriticalPair &criticals, Formula f1, Formula f2, Signature &s) {
  cout << "Uzimamo kriticne parove za " << f1 << " i " << f2 << endl;
  cout << endl;
  Atom *a1 = (Atom*) f1.get();
  Atom *a2 = (Atom*) f2.get();

  vector<Term> f1Terms;
  vector<Term> f2Terms;
  
  f1Terms = a1->getOperands();
  f2Terms = a2->getOperands();
  
  TermPairs termPairs;
  Substitution sub;
  
// l1' neka je to podterm l1 koji nije promenljivai neka je O najopstiji unifikator termova l1' i l2

  vector<Term>::iterator l1= f1Terms.begin();
  Term l1_prim = getSubterm (*l1);
  vector<Term>::iterator r1 = ++f1Terms.begin();

  vector<Term>::iterator l2= f2Terms.begin();
  vector<Term>::iterator r2 = ++f2Terms.begin();
  //Term r2 = *(++l2);

  termPairs.push_back(make_pair(l1_prim, *l2));
  
  cout << "***********Proces unifikacije***************"<<endl;
  if (unify(termPairs, sub) == false)
	   cout << "Ne moze da nadje najop. unifikator" << endl;
  
  // l1 -> r1 je iz f1
  // l2 -> r2 je iz f2
	
 FunctionTerm* ftl1 = (FunctionTerm*) (*l1).get();
 FunctionTerm* ftr1 = (FunctionTerm*) (*r1).get();
 
 FunctionTerm* ftl1p = (FunctionTerm*) l1_prim.get();
 VariableTerm* vtr2 = (VariableTerm*) (*r2).get();

  //for(std::vector<pair<Variable, Term>>::iterator iter = sub.begin(); iter != sub.end(); iter++)
//{
  std::vector<pair<Variable, Term>>::iterator iter = sub.begin();
	// primenjujemo dobijen najopstiji unifikator
	ftl1->substitute(iter->first, iter->second);
	ftr1->substitute(iter->first, iter->second);

	ftl1p->substitute(iter->first, iter->second);
	vtr2->substitute(iter->first, iter->second);
	
	ftl1->substitute(vtr2->getVariable(), *((Term*)ftl1p));
	
  /*TODO IZDEBAGOVATI OVOOOOO!!!!!!!!!!!!!! */
	//Term* aaa = (dynamic_cast<Term*>(ftr1));
	//Term* bbb = (dynamic_cast<Term*>(ftl1));
  Term nula = make_shared<FunctionTerm> (s, "0");
  Term a = make_shared<VariableTerm>("a");
  Term s_nula = make_shared<FunctionTerm> (s, "s", vector<Term> {nula});
	Term u1 = make_shared<FunctionTerm> (s, "+", vector<Term> {a, s_nula});
  Term u2 = make_shared<FunctionTerm> (s, "s", vector<Term> {a});

  cout << endl;
  cout << "***********kriticni par***********" << endl;
  cout << "<" << u1 << "," << u2 << ">" << endl;

//}
  criticals.t1 = u1;
  criticals.t2 = u2;
  //term l1[l1'->O(l2)] odredjuje kriticni par <O(r1), O(l1)[O(l1')->O(r2)]>
}

void knut_bendix (RewriteSystem &system, Signature &s){
  std::cout << "****Knut Bendix procedura upotpunjavanja****" << std::endl;
  cout << endl << endl;

  //algoritam

/*  //za setnju kroz pravila
  RewriteSystem s2 = system;
  RewriteSystem::iterator it2 = s2.begin();
  it2++;

  //uzimamo prvu relaciju
  RewriteSystem::iterator it = system.begin();

  for (proci kroz sve relacijeSve){

    //getAllCriticalPairs(*it1, *it2);
    //uzeti prvi kriticni i kombinovati sa svim ostalim
    
    for (svaki <u1, u2> )
      u1_pom = normalna_forma(u1);
      u2_pom = normalna_forma(u2);

      if(u1_pom != u2_pom && duzine_jednake(u1_pom, u2_pom))
        return fail;
      else if(u1_pom > u2_pom)
        relacija = relacija.append(rewrite(u1_pom, u2_pom));
      else if(u2_pom > u1_pom)
        relacija = relacija.append(rewrite(u2_pom, u1_pom));
  
      relacija ++;
  }*/

  RewriteSystem::iterator it = system.begin();
  RewriteSystem::iterator it2 = ++(system.begin());

  //uzimamo sve kriticne parove za trenutnu kombinaciju rewrite relacija      
  /*TODO PROCI KROZ SVE PAROVE, MI IH IMAMO SAMO 2 ZA SADA*/
  CriticalPair criticals;

  getAllCriticalPairs(criticals, *it, *it2, s);

  Formula u1 = make_shared<Atom> (s, "u1", vector<Term>{criticals.t1});
  Formula u2 = make_shared<Atom> (s, "u2", vector<Term>{criticals.t2});

  //nema smisla bas ali ajde
  Formula u1_normal = u1->simplify()->nnf()->prenex()->skolem(s);
  Formula u2_normal = u2->simplify()->nnf()->prenex()->skolem(s);

  if (u1_normal->equalTo(u2_normal)){
    cout << "Sistem ne moze biti dopunjen" << endl;
    return;
  }

  /*TODO  odraditi funkciju za dobro uredjenje termova */
  Formula rewrite = make_shared<Atom>(s, "rewrite", vector<Term> {criticals.t1, criticals.t2});
  system.insert(rewrite);
}


int main()
{

  /* Definisemo strukturu */
  Signature s;

  /* Dodajemo funkcijske i predikatske simbole */
  s.addFunctionSymbol("0", 0);
  s.addFunctionSymbol("1", 0);
  s.addFunctionSymbol("+", 2);
  s.addFunctionSymbol("*", 2);
  s.addFunctionSymbol("s", 1);
  s.addPredicateSymbol("even", 1);
  s.addPredicateSymbol("odd", 1);
  s.addPredicateSymbol("=", 2);
  s.addPredicateSymbol("<=", 2);
  s.addPredicateSymbol("u1", 1);
  s.addPredicateSymbol("u2", 1);

  //rewrite predicate symbol
  s.addPredicateSymbol("rewrite", 2);

  /* Primeri termova i formula */

  Term t0 = make_shared<FunctionTerm>(s, "0");
  Term t1 = make_shared<FunctionTerm>(s, "1");
  Term tx = make_shared<VariableTerm>("x");
  Term ty = make_shared<VariableTerm>("y");
  Term ta = make_shared<VariableTerm>("a");
  Term tb = make_shared<VariableTerm>("b");  
  Term xpy = make_shared<FunctionTerm>(s, "+", vector<Term> {tx, ty});
  Term t_nn = make_shared<FunctionTerm> (s, "*", vector<Term> {ta, tb});

  /* Termovi koji ucestvuju u rewrite */
  Term s_fn = make_shared<FunctionTerm> (s, "s", vector<Term> {xpy});
  Term s_y = make_shared<FunctionTerm> (s, "s", vector<Term> {ty});
  Term s_xpsy = make_shared<FunctionTerm> (s, "+", vector<Term> {s_y, tx});
  Term apzero = make_shared<FunctionTerm> (s, "+", vector<Term> {ta, t0});

 //formule koja predstavljaju rewriting
  Formula rewrite1 = make_shared<Atom>(s, "rewrite", vector<Term> {s_fn, s_xpsy});
  Formula rewrite2 = make_shared<Atom>(s, "rewrite", vector<Term> {apzero, ta});

  RewriteSystem system;

  system.insert(rewrite1);
  system.insert(rewrite2);

  //printing set of rewrites
  cout << "***********Sistem za prezapisivanje***********" << endl;
  cout << "{ ";
  for (RewriteSystem::iterator it = system.begin(); it != system.end(); it++) {

      cout << *it << " ";
  }
  cout << " }";

  cout << endl << endl << endl;

  knut_bendix (system, s);

  cout << endl << endl << endl;
  cout << "**********Ispisujemo dopunjen sistem************" << endl;
  //printing completation set of rewrites
  cout << "{ ";
  for (RewriteSystem::iterator it = system.begin(); it != system.end(); it++) {

      cout << *it << " ";
  }
  cout << " }" << endl;
  cout << endl << endl << endl;

	return 0;
}







