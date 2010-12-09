
import collections

FieldMeta = collections.namedtuple('FieldMeta', 'units format title')

# Wrapper for namedtuple which adds on extra field metadata
# in a cls.fields dict.
def metatuple(*fieldstrs):
    fieldmetas = {}
    fieldnames = []
    for fieldmeta in map(lambda s: s.split(' ', 4), fieldstrs):
        if len(fieldmeta) == 1: fieldmeta.append('')  # Default units
        if fieldmeta[1] == '-': fieldmeta[1] = ''
        if len(fieldmeta) == 2: fieldmeta.append("%.2f")  # Default format
        if len(fieldmeta) == 3: 
            # Default title based on field name.
            fieldmeta.append(fieldmeta[0].replace('_', ' ').title())
        fieldnames.append(fieldmeta[0])
        fieldmetas[fieldmeta[0]] = FieldMeta(*fieldmeta[1:])
        
    tuplename = "p%d_" % abs(hash(tuple(fieldnames)))
    # print fieldnames
    nt = collections.namedtuple(tuplename, fieldnames)
    nt.fields = fieldmetas
    return nt
    

class Fields(object):

    def __str__(self):
        lines = []
        for field in self._fields:
            meta = self.fields[field]
            value = getattr(self, field)
            #if isinstance(value, Fields):
                # Special case for the Payload members of Message.
            #    value = str(value)

            one = lambda value: "%s %s" % (meta.format, meta.units) % value
            if isinstance(value, list):
                if len(value) > 0:
                    valuestr = ', '.join(map(one, value))
                else:
                    valuestr = '<none>'
            else:
                valuestr = one(value)

            line = "%s: %s" % (meta.title, valuestr)
            lines.append(line)

        return "\n".join(lines)

    def __repr__(self):
        '''Get the namedtuple's repr, and replace the name with the current classname.
        This permits payloads to inherit from intermediary abstract payloads.'''
        parent_repr = super(Fields, self).__repr__()
        name, sep, fields = parent_repr.partition('_(')
        return "%s(%s" % (self.__class__.__name__, fields)

    def hex(self):
        return ' '.join(map(lambda c: "%02X" % c, self.data()))

    def check(self, fields):
        names = fields.split()
        return self.Check(self.__class__.__name__, names, 
                          map(lambda name: getattr(self, name), names))

    class Check(object):
        def __init__(self, subjname, names, values):
            self.names = names
            self.values = values
            self.subjname = subjname

        def _check(self, constraint, explanation):
            for name, value in zip(self.names, self.values):
                if constraint(value):
                    e = "%s initialized with %s=%s: %s" % (self.subjname, name, repr(value), explanation)
                    raise ValueError(e)

        def range(self, lowerbound, upperbound):
            self._check(lambda x: x < lowerbound or x > upperbound,
                        "Outside of allowed range [%0.1f,%0.1f]" % 
                        (lowerbound, upperbound))

        def length(self, lowerbound, upperbound):
            self._check(lambda x: len(x) < lowerbound, 
                        "Length shorter than %d" % lowerbound)
            self._check(lambda x: len(x) > upperbound, 
                        "Length longer than %d" % upperbound)

        def each(self):
            subnames = []
            subvalues = []
            for name, value in zip(self.names, self.values):
                # Confirm each of these is a list, then break it down and populate the sub-Check 
                # object with the names and values of the individual items.
                self._check(lambda x: not isinstance(x, list), "Not a list")
                subnames += map(lambda (index, subvalue): "%s[%s]" % (name, repr(index)), enumerate(value))
                subvalues += value
            return self.__class__(self.subjname, subnames, subvalues)

        def ascii(self):
            self._check(lambda x: not all(ord(c) < 128 for c in x), 
                        "String contains non-ascii characters.")
