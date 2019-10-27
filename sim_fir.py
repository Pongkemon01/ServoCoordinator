import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import signal

#f0 = 0.15
f0 = 10.0
fs = 100.0
dur = 5                      #seconds
def genSine(f0,fs,dur):
    t = np.linspace(0, dur, dur*fs+1)
    sinewave = np.sin(2*np.pi*f0*t)
    return t, sinewave

def genNoise(dur):
    noise = np.random.normal(0,1,int(fs*dur)+1)
    noise = noise + 4
    return noise

ORDER=5     # Odd Number please
H_ORDER = ( ORDER + 1 ) / 2

buffer = np.zeros(ORDER)
f1 = 0.37*f0
f2 = f0/0.37
fc1 = f1 * 0.1
fc2 = f2 * 0.1
bands = [0, f1 - fc1, f1, f2, f2 + fc2, 0.5*fs]
print(bands)
co = signal.remez(ORDER, bands, [0, 1, 0], Hz=fs)
#co = np.array((0.0219, 0.1407, 0.3626, 0.3626, 0.1407, 0.0219))
print(H_ORDER)
#print(co)
def FIR(s):
    i = 1
    while( i < ORDER - 1 ):
        buffer[ORDER - i] = buffer[ORDER - i - 1] + (s * co[i - 1])
        i = i + 1
    buffer[0] = s * co[ORDER - 1]

    return( buffer[ ORDER - 1 ] )

symtrbuf = np.zeros(ORDER + 1)
def SymTransposeFIR(s):
    # Requires a temporary result buffer as we must retain the previous results till the end of calculation
    tbuf = np.zeros(ORDER)
    pr = s * co[int(H_ORDER - 1)]
    symtrbuf[ORDER] = symtrbuf[ORDER - 1] + pr
    tbuf[0] = pr 
    i = 1
    while( i < H_ORDER):
        pr = s * co[int(H_ORDER - 1 - i)]
        tbuf[ORDER - i] = symtrbuf[ORDER - i - 1] + pr
        tbuf[i] = symtrbuf[i - 1] + pr
        i = i + 1
    i = 0
    # Copy the results to the real storage
    while( i < ORDER):
        symtrbuf[i] = tbuf[i]
        i = i + 1

    return( symtrbuf[ORDER] )

symbuf = np.zeros(ORDER + 1)

if __name__ == '__main__':
    t, sum = genSine(f0,fs,dur)
    noise = genNoise(dur)
    #sum = sum + noise
    fil = np.array([])
    fil2 = np.array([])
    for s in sum :
        fil2 = np.append( fil2, SymTransposeFIR(s)) 
        fil = np.append( fil, FIR(s))

    rawline = plt.plot(t, sum, label='Input')
    filline = plt.plot(t, fil, label='Directed Transposed FIR')
    fil2line = plt.plot(t, fil2, label='Linear-phase Transpose FIR')
    plt.legend()

    plt.show()

