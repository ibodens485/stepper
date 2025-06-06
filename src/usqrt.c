#define BITSPERLONG (sizeof(long) * 8)
#define TOP2BITS(x) ((x & (3L << (BITSPERLONG - 2))) >> (BITSPERLONG - 2))

unsigned long usqrt(unsigned long x) {
    unsigned long a = 0L; /* accumulator      */
    unsigned long r = 0L; /* remainder        */
    unsigned long e = 0L; /* trial product    */

    for (int i = 0; i < BITSPERLONG / 2; i++) { /* NOTE 1 */
        r = (r << 2) + TOP2BITS(x);
        x <<= 2; /* NOTE 2 */
        a <<= 1;
        e = (a << 1) + 1;
        if (r >= e) {
            r -= e;
            a++;
        }
    }
    return a;
}

#define BITSPERLL (sizeof(unsigned long long) * 8)
#define TOP2BITS_LL(x) (((x) & (3ULL << (BITSPERLL - 2))) >> (BITSPERLL - 2))

unsigned long usqrt_ll(unsigned long long x) {
    unsigned long a = 0L;        /* accumulator      */
    unsigned long long r = 0ULL; /* remainder        */
    unsigned long e = 0L;        /* trial product    */

    for (int i = 0; i < BITSPERLL / 2; i++) {
        r = (r << 2) + TOP2BITS_LL(x);
        x <<= 2;
        a <<= 1;
        e = (a << 1) + 1;
        if (r >= e) {
            r -= e;
            a++;
        }
    }
    return a;
}