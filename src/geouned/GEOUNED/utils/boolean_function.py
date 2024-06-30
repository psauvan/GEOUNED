#
# Define the class storing Boolean expression of the cell definition
#

import logging
import re

logger = logging.getLogger("general_logger")

mostinner = re.compile(r"\([^\(^\)]*\)")  # identify most inner parentheses
number = re.compile(r"(?P<value>[-+]?\d+)")  # identify signed integer and record its value in <value>
mix = re.compile(r"(?P<value>([-+]?\d+|\[0+\]))")  # identify signed integer or [000...] pattern. Record the value.
TFX = re.compile(r"(?P<value>[FTXo]+)")  # identify pattern incluinding F,T,X, or o sequence ( in any order).
PValue = re.compile(r"P\d+")  # identify pattern "P" + integer pattern (e.g. P3915).
NValue = re.compile(r"N\d+")  # identify pattern "N" + integer pattern (e.g. N3358).
conversion = {
    "T": True,
    "F": False,
    "X": None,
}  # associate "T", "F", "X" with associated Boolean value (or None for X)


class bsurface(int):
    """Boolean Surface class"""

    def __new__(cls, label: int, s1: int = None, s2: int = None, op: str = None, aux: bool = False):
        return super(bsurface, cls).__new__(cls, label)

    def __neg__(self):
        if self.bool:
            bs = self.copy()
            bs.s1 = not bs.s1
            bs.label = -bs.label
            return bs
        elif self.s2 is None:
            return bsurface(-self.label, -self.s1, aux=self.aux)
        else:
            return bsurface(-self.label, -self.s1, -self.s2, self.op_comp(), aux=self.aux)

    def __pos__(self):
        return self

    def __abs__(self):
        if self.label >= 0:
            return self.copy()
        else:
            return -self.copy()

    def __str__(self):
        if self.s2 is None:
            return str(f" {self.s1}")
        else:
            if self.op == "AND":
                return f"AND[{self.s1} {self.s2}]"
            else:
                return f"OR[{self.s1} {self.s2}]"

    def __init__(self, label: int, s1: int = None, s2: int = None, op: str = None, aux: bool = False):

        self.label = label
        if s1 is None:
            self.s1 = label
        else:
            self.s1 = s1

        self.s2 = s2
        self.op = op
        self.aux = False
        self.bool = False

        if self.s2 is not None:
            assert self.s2 != 0, "s2 must not be zero"
            assert self.op in ("OR", "AND"), 'operator should be "OR" or "AND"'
        else:
            self.op = None

    def compare(self, s):
        if abs(self.s1) != abs(s.s1):
            return (False, False)
        if self.s2 is None and s.s2 is None:
            return (True, self.s1 != s.s1)

        if abs(self.s2) != abs(s.s2):
            return (False, False)

        if (self.s1 == s.s1) and (self.s2 == s.s2) and (self.op == s.op):
            return (True, False)
        scomp = -s
        if (self.s1 == scomp.s1) and (self.s2 == scomp.s2) and (self.op == scomp.op):
            return (True, True)
        return (False, False)

    def copy(self):
        if self.bool:
            bs = bsurface(self.label)
            if self.s1:
                bs.set_true()
                return bs
            else:
                bs.set_false()
                return bs

        elif self.s2 is None:
            return bsurface(self.label, self.s1, aux=self.aux)
        else:
            return bsurface(self.label, self.s1, self.s2, self.op, aux=self.aux)

    def set_true(self):
        self.bool = True
        self.s1 = True
        self.s2 = None
        self.op = None

    def set_false(self):
        self.bool = True
        self.s1 = False
        self.s2 = None
        self.op = None

    def op_comp(self):
        return "AND" if self.op == "OR" else "OR"

    def add_surface(self, s2: int, op: str):

        assert op in ("OR", "AND"), 'operator should be "OR" or "AND"'
        assert s2 != 0, "s2 must not be zero"

        self.s2 = s2
        self.op = op

    def get_BoolSequence(self, aux=False):
        if self.s2 is None:
            return self

        seq = BoolSequence(self.op)
        seq.append(bsurface(self.s1, aux=aux))

        if type(self.s2) is int:
            seq.append(bsurface(self.s2, aux=True))

        else:
            subSeq = self.s2.get_BoolSequence(aux=True)
            seq.append(subSeq)

        seq.group_single()
        seq.simplify()
        return seq


class BoolSequence:
    """Class storing Boolean expression and operating on it"""

    def __init__(self, definition=None, operator=None):
        if definition:
            self.elements = []
            self.set_def(definition)
        else:
            self.elements = []
            self.operator = operator
            self.level = 0

    def __str__(self):
        out = f"{self.operator}["
        if type(self.elements) is bool:
            return " True " if self.elements else " False "
        for e in self.elements:
            out += e.__str__()

        out += "] "
        return out

    def append(self, *seq):
        """Append a BoolSequence Objects. seq may be :
        - An iterable containing allowed BoolSequence Objects
        - A BoolSequence object
        - A bsurface object"""

        for s in seq:
            if type(s) is bsurface:
                if not s.bool:
                    level = -1
                    if s in self.elements:
                        continue
                    elif -s in self.elements:
                        self.level = -1
                        self.elements = True if self.operator == "OR" else False
                        return
            elif type(s) is bool:
                if self.operator == "AND" and s or self.operator == "OR" and not s:
                    continue
                else:
                    self.elements = s
                    self.level = -1
                    return
            else:
                level = s.level
                if type(s.elements) is bool:
                    if self.operator == "AND" and not s.elements or self.operator == "OR" and s.elements:
                        self.level = -1
                        self.elements = s.elements
                        return
                    else:
                        continue

            self.elements.append(s)
            self.level = max(self.level, level + 1)

    def assign(self, seq):
        """Assign the BoolSequence Seq to the self instance BoolSequence"""
        if type(seq) is bool:
            self.operator == "AND"
            self.elements = seq
            self.level = -1
            return

        self.operator = seq.operator
        self.elements = seq.elements
        self.level = seq.level

    def update(self, seq, pos):
        if len(pos) == 0:
            self.assign(seq)
            return
        elif len(pos) == 1:
            base = self
        else:
            base = self.get_element(pos[:-1])

        indexes = pos[-1]
        indexes.sort()
        for i in reversed(indexes):
            del base.elements[i]

        if type(seq.elements) is bool:
            base.elements = seq.elements
            base.level = -1
        else:
            base.append(seq)
            base.join_operators()
        self.clean(self_level=True)
        return

    def get_element(self, pos):
        if len(pos) == 1:
            return self.elements[pos[0]]
        else:
            return self.elements[pos[0]].get_element(pos[1:])

    def copy(self):
        cp = BoolSequence()
        cp.operator = self.operator
        cp.level = self.level
        if type(self.elements) is bool:
            cp.elements = self.elements
        else:
            for e in self.elements:
                cp.elements.append(e.copy())
        return cp

    # TODO rename to snake case, care as multiple functions with same name
    def get_complementary(self):
        c = BoolSequence(operator=self.comp_operator())
        c.level = self.level

        if self.level == 0:
            for e in self.elements:
                c.elements.append(-e)
            return c
        else:
            self.group_single()
            for e in self.elements:
                c.elements.append(e.get_complementary())
            return c

    def comp_operator(self):
        if self.operator == "AND":
            return "OR"
        else:
            return "AND"

    def simplify(self, CT=None, depth=0):
        """Simplification by recursive calls to the inner BoolSequence objects."""
        if self.level > 0:
            for seq in self.elements:
                seq.simplify(CT, depth + 1)
            self.clean()
            self.join_operators()
            self.level_update()

        if type(self.elements) is not bool and (self.level > 0 or len(self.elements) > 1):
            levIn = self.level
            self.simplify_sequence(CT)

            if self.level > levIn and depth < 10:
                self.simplify(CT, depth + 1)

        if self.level == 0:
            self.elements = list(set(self.elements))

    def simplify_sequence(self, CT=None):
        """Carry out the simplification process of the BoolSequence."""
        if self.level < 1 and CT is None:
            self.clean()
            return

        surf_names = self.get_surfaces_numbers()
        if not surf_names:
            return

        newNames = surf_names
        for val_name in surf_names:
            if val_name in newNames:
                if CT is None or val_name.aux:
                    true_set = {val_name: True}
                    false_set = {val_name: False}
                else:
                    true_set, false_set = CT.get_constraint_set(val_name)

                if not self.do_factorize(val_name, true_set, false_set):
                    continue
                self.factorize(val_name, true_set, false_set)
                if type(self.elements) is bool:
                    return
                newNames = self.get_surfaces_numbers()

    def do_factorize(self, val_name: bsurface, true_set: dict, false_set: dict):
        """For level 0 sequence check if the factorization would lead to a simplification."""
        if self.level > 0:
            return True
        if true_set is None and false_set is None:
            logger.info(f"{val_name} is not true nor false")
            return False
        if true_set is None or false_set is None:
            return True

        val_set = self.get_surfaces_numbers()
        t_set = set(true_set.keys()) & val_set
        f_set = set(false_set.keys()) & val_set

        if len(t_set) == 1 and len(f_set) == 1:
            return False

        value = None
        for val in self.elements:
            if abs(val) == val_name:
                value = val
                break

        if value is None:
            return False

        if len(t_set) == 1:
            if self.operator == "AND":
                # if value > 0 and t_set[val_name] or value < 0 and not t_set[val_name] : return False
                if value > 0:
                    return False  # TrueSet[Valname] always True
            else:
                # if value < 0 and t_set[val_name] or value > 0 and not t_set[val_name] : return False
                if value < 0:
                    return False

        elif len(f_set) == 1:
            if self.operator == "AND":
                # if value > 0 and f_set[val_name] or value < 0 and not f_set[val_name] : return False
                if value < 0:
                    return False
            else:
                # if value < 0 and f_set[val_name] or value > 0 and not f_set[val_name] : return False
                if value > 0:
                    return False

        return True

    # check if level 0 sequence have opposite value a & -a = 0  , a|-a = 1
    # return the value of the sequence None(unknown), True, False
    def check(self, level0=False):
        """Check BoolSequence in level 0 have oposite values  a & -a = 0  , a|-a = 1."""
        if type(self.elements) is bool:
            return self.elements
        if self.level == 0:
            signed_surf = set(self.elements)
            surf_name = self.get_surfaces_numbers()
            if len(signed_surf) == len(surf_name):
                return None  # means same surface has not positive and negative value
            elif self.operator == "AND":
                self.elements = False
                self.level = -1
                return False
            else:
                self.elements = True
                self.level = -1
                return True
        elif not level0:
            self.group_single()
            none_val = False
            for e in reversed(self.elements):
                e.check()
                if type(e.elements) is bool:
                    bres = self.bool_element_value(e.elements)
                    if bres is None:
                        self.elements.remove(e)
                    else:
                        return bres
                else:
                    none_val = True

            if none_val:
                return None
            elif self.operator == "AND":
                self.level = -1
                self.elements = True
                return True
            else:
                self.level = -1
                self.elements = False
                return False

    def substitute(self, var, val):
        """Substitute in the BoolSequence the variable "var" by the value "val".
        "val" can be an Boolean value or an integer representing another surface variable.
        """
        if val is None:
            return
        if type(self.elements) is bool:
            return
        name = abs(var)
        ic = len(self.elements)
        for e in reversed(self.elements):
            ic -= 1
            if type(e) is bsurface:
                if not e.bool:
                    if abs(e) == name:
                        if type(val) is bsurface:
                            if name == e:
                                self.elements[ic] = val
                            else:
                                self.elements[ic] = -val
                        else:
                            if name == e:
                                bool_value = val
                            else:
                                bool_value = not val

                            bres = self.bool_element_value(bool_value)
                            if bres is None:
                                self.elements.remove(e)
                            else:
                                return
            else:
                e.substitute(var, val)
                if type(e.elements) is bool:
                    bres = self.bool_element_value(e.elements)
                    if bres is None:
                        self.elements.remove(e)
                    else:
                        return

        if self.elements == []:
            self.elements = True if self.operator == "AND" else False
            self.level = -1
            return

        self.clean(self_level=True)
        self.check(level0=True)
        self.join_operators(self_level=True)

    def clean(self, self_level=False):
        """Remove sequences whom elements are boolean values instead of list."""
        if type(self.elements) is bool:
            return self.elements
        for e in reversed(self.elements):
            if type(e) is bsurface:
                if not e.bool:
                    continue
                else:
                    bres = self.bool_element_value(e.s1)
                    if bres is None:
                        self.elements.remove(e)
                        continue
                    else:
                        return bres
            elif type(e) is bool:
                continue

            eVal = e if self_level else e.clean()
            if type(eVal) is not bool:
                eVal = eVal.elements
            else:
                bres = self.bool_element_value(eVal)
                if bres is None:
                    self.elements.remove(e)
                else:
                    return bres

        if self.elements == []:
            self.elements = True if self.operator == "AND" else False
            self.level = -1
            return self.elements
        else:
            if len(self.elements) == 1:
                if type(self.elements[0]) is bool:
                    self.elements = self.elements[0]
                    self.level = -1
            return self

    # TODO rename to snake case, care as multiple functions with same name
    def join_operators(self, self_level=False):
        """Join redundant operators in found in the sequence."""
        if type(self.elements) is bool:
            return
        self.clean(self_level=True)
        self.level_update()
        self.group_single()
        if self.level < 1:
            return

        ANDop = []
        ORop = []

        for e in self.elements:
            if e.operator == "AND":
                ANDop.append(e)
            else:
                ORop.append(e)

        if len(ANDop) > 1 and self.operator == "AND":
            newSeq = BoolSequence(operator="AND")
            for s in ANDop:
                newSeq.elements.extend(s.elements)
                self.elements.remove(s)
            newSeq.level_update()
            self.append(newSeq)

        elif len(ORop) > 1 and self.operator == "OR":
            newSeq = BoolSequence(operator="OR")
            for s in ORop:
                newSeq.elements.extend(s.elements)
                self.elements.remove(s)
            newSeq.level_update()
            self.append(newSeq)

        if self.level > 0 and len(self.elements) == 1:
            self.operator = self.elements[0].operator
            self.elements[:] = self.elements[0].elements[:]
            self.level -= 1
            self.join_operators()

        if self.level == 0:
            self.check()
            return

        if not self_level:
            if type(self.elements) is bool:
                return
            for e in self.elements:
                e.join_operators()

    def get_sub_sequence(self, setIn):
        if type(setIn) is set:
            val_set = setIn
        elif type(setIn) is bsurface:
            val_set = {setIn}
        else:
            val_set = set(setIn.keys())

        if self.level == 0:
            return ([], self)

        position = []
        subSeq = BoolSequence(operator=self.operator)

        for pos, e in enumerate(self.elements):
            surf = e.get_surfaces_numbers()
            if len(surf & val_set) != 0:
                subSeq.append(e)
                position.append(pos)

        if len(position) == 1 and subSeq.elements[0].level > 0:
            subList, subSeq = subSeq.elements[0].get_sub_sequence(val_set)
            subList.insert(0, position[0])
        else:
            subList = [position]

        return subList, subSeq

    def factorize(self, valname: bsurface, true_set: dict, false_set: dict):
        """Make the factorization of the Sequence on variable valname using Shannon's theorem."""
        if true_set is None:  # valname cannot take True value
            falseFunc = self.evaluate(false_set)
            self.assign(falseFunc)
            return True

        if false_set is None:  # valname cannot take false value
            trueFunc = self.evaluate(true_set)
            self.assign(trueFunc)
            return True

        val_set = set(true_set.keys())
        val_set.update(false_set.keys())
        pos, subSeq = self.get_sub_sequence(val_set)
        updt = True
        if len(pos) == 0:
            subSeq = self
            updt = False

        trueFunc = subSeq.evaluate(true_set)

        falseFunc = subSeq.evaluate(false_set)

        if trueFunc is False:
            newSeq = BoolSequence(operator="AND")
            if falseFunc is True:
                newSeq.append(-valname)
            elif falseFunc is False:
                newSeq.elements = False
                newSeq.level = -1
            else:
                newSeq.append(-valname, falseFunc)
                newSeq.join_operators(self_level=True)

            if updt:
                self.update(newSeq, pos)
            else:
                self.assign(newSeq)
            return True

        elif trueFunc is True:
            newSeq = BoolSequence(operator="OR")
            if falseFunc is True:
                newSeq.elements = True
                newSeq.level = -1
            elif falseFunc is False:
                newSeq.append(valname)
            else:
                newSeq.append(valname, falseFunc)
                newSeq.join_operators(self_level=True)

            if updt:
                self.update(newSeq, pos)
            else:
                self.assign(newSeq)
            return True

        if falseFunc is False:
            newSeq = BoolSequence(operator="AND")
            if trueFunc is True:
                newSeq.append(valname)
            elif trueFunc is False:
                newSeq.elements = False
                newSeq.level = -1
            else:
                newSeq.append(valname, trueFunc)
                newSeq.join_operators(self_level=True)
            if updt:
                self.update(newSeq, pos)
            else:
                self.assign(newSeq)
            return True

        elif falseFunc is True:
            newSeq = BoolSequence(operator="OR")
            if trueFunc is True:
                newSeq.elements = True
                newSeq.level = -1
            elif trueFunc is False:
                newSeq.append(-valname)
            else:
                newSeq.append(-valname, trueFunc)
                newSeq.join_operators(self_level=True)
            if updt:
                self.update(newSeq, pos)
            else:
                self.assign(newSeq)
            return True

    def evaluate(self, valueSet):
        """Return the result of the evaluation of the BoolSequence given the known values of the variables in "valueSet".
        Result can be a Boolean value or the reduced expresion of the BoolSequence."""
        if type(self.elements) is bool:
            return self.elements
        self.group_single()
        newSeq = self.copy()
        for name, value in valueSet.items():
            newSeq.substitute(name, value)
            if type(newSeq.elements) is bool:
                return newSeq.elements

        return newSeq.elements if type(newSeq.elements) is bool else newSeq

    def set_def(self, expression):
        """Set the expression of the Boolean function in the BoolSequence instance.
        "expresion" is the string object. The definition should have MCNP syntax cell definition.
        """
        terms, operator = outer_terms(expression)
        self.operator = operator
        self.level = 0
        lev0Seq = set()
        lev0SeqAbs = set()
        for t in terms:
            if is_integer(t):
                val = int(t.strip("(").strip(")"))
                bval = bsurface(val)
                lev0Seq.add(bval)
                lev0SeqAbs.add(abs(bval))
                # self.elements.append(int(t.strip('(').strip(')')))
            else:
                x = BoolSequence(t)
                self.level = max(x.level + 1, self.level)
                self.append(x)

        # check if in integer sequence there is surface sequence s -s
        if len(lev0Seq) != len(lev0SeqAbs):
            if self.operator == "AND":
                self.elements = False
            else:
                self.elements = True
            self.level = -1
        else:
            self.append(*lev0Seq)

        self.group_single()

    def group_single(self):
        """group integers found in Sequence with level > 1.

        (e.g. change AND[1 2 3 OR[2 4]] to AND[ AND[1 2 3] OR[2 3]] )."""
        if self.level == 0:
            if len(self.elements) == 1:
                if type(self.elements[0]) is bool:
                    self.elements = self.elements[0]
                    self.level = -1
            return
        if type(self.elements) is bool:
            return
        group = []
        for e in reversed(self.elements):
            if type(e) is bsurface:
                if e.bool:
                    if e.s1 and self.operator == "OR":
                        group = True
                        break
                    elif not e.s1 and self.operator == "AND":
                        group = False
                        break
                else:
                    group.append(e)
                self.elements.remove(e)

            elif type(e.elements) is bool:
                if e.elements and self.operator == "OR":
                    group = True
                    break
                elif not e.elements and self.operator == "AND":
                    group = False
                    break
                self.elements.remove(e)

            elif e.level == 0 and len(e.elements) == 1:
                if type(e.elements[0]) is bool:
                    group = e.elements[0]
                    break
                else:
                    group.append(e.elements[0])
                    self.elements.remove(e)

        if type(group) is bool:
            self.elements = group
            self.level = -1
            return

        elif len(group) == 0:
            if len(self.elements) == 0:
                self.level = -1
                if self.operator == "AND":
                    self.elements = True
                else:
                    self.elements = False
            return

        seq = BoolSequence()
        seq.elements.extend(group)
        seq.operator = self.operator
        seq.level = 0
        self.elements.insert(0, seq)

    def get_surfaces_numbers(self):
        """Return the list of all surfaces in the BoolSequence definition."""
        if type(self.elements) is bool:
            return tuple()
        surf = set()
        for e in self.elements:
            if type(e) is bsurface:
                surf.add(abs(e))
            else:
                surf.update(e.get_surfaces_numbers())
        return surf

    def level_update(self):
        """Update the level value of the BoolSequence."""
        if type(self.elements) is bool:
            self.level = -1
            return

        self.level = 0
        for e in self.elements:
            if type(e) is BoolSequence:
                e.level_update()
                self.level = max(e.level + 1, self.level)

    def bool_element_value(self, b: bool):
        """return the self.elements value when surface value is True or False ."""
        if b and self.operator == "OR":
            self.elements = True
            self.level = -1
            return True
        elif not b and self.operator == "AND":
            self.elements = False
            self.level = -1
            return False
        else:
            return None


def outer_terms(expression, value="number"):
    """Return the list and the boolean operator of the outter terms of the expression."""
    if value == "number":
        # reValue = number
        reValue = mix
        nullVal = "0"
    else:
        reValue = TFX
        nullVal = "o"

    expr = expression

    # Loop until no redundant parentheses are found
    cont = True

    while cont:
        # Loop over most inner parentheses
        pos = 0
        cont = False
        while True:
            m = mostinner.search(expr, pos)
            if not m:
                break
            cont = True
            if redundant(m, expr):
                # remove redundant parentheses
                expr = expr[: m.start()] + " " + expr[m.start() + 1 : m.end() - 1] + " " + expr[m.end() :]
            else:
                # replace no redundant parentheses by 0 and : by ;
                zeros = "[" + nullVal * (m.end() - m.start() - 2) + "]"
                expr = expr[: m.start()] + zeros + expr[m.end() :]

            pos = m.end()

    if ":" in expr:
        terms = []
        pos = 0
        while True:
            new_pos = expr.find(":", pos)
            if new_pos == -1:
                terms.append(expression[pos:].strip())
                break
            terms.append(expression[pos:new_pos].strip())
            pos = new_pos + 1
        return (terms, "OR")
    else:
        terms = []
        pos = 0
        while True:
            m = reValue.search(expr, pos)
            if not m:
                break
            terms.append(expression[m.start() : m.end()])
            pos = m.end()
        return (terms, "AND")


def redundant(m, geom):
    """Check if the inner parentheses are redundant."""
    term = m.group()

    # Find first valid character at the left of the  parenthese
    left_ok = True
    left = m.start() - 1
    while left > -1:
        if geom[left] in ("\n", "C", "$", " "):
            left -= 1
        else:
            if geom[left] not in ("(", ":"):
                left_ok = False
            break

    # check if no ':' (or) are inside the parenthese
    # if not, parentheses are redundants
    if term.find(":") == -1:
        return True

    # Find first valid character at the right of the  parenthese
    right_ok = True
    right = m.end()
    while right < len(geom):
        if geom[right] in ("\n", "C", "$", " "):
            right += 1
        else:
            if geom[right] not in (")", ":"):
                right_ok = False
            break

    # if parentheses are like:
    # {( or : } ( ....... ) {) or :}
    # parentheses are redundants

    if left_ok and right_ok:
        return True
    else:
        return False


def is_integer(x):
    try:
        int(x.strip("(").strip(")"))
        return True
    except:
        return False
