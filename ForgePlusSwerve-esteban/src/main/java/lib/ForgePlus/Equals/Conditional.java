package lib.ForgePlus.Equals;

public class Conditional {
    
    @FunctionalInterface
    private interface Chooser<T>{
        T from(T a, T b, boolean condition);        
    }

    public static<T> T chooseBetween(T a, T b, boolean condition) {
        Chooser<T> c = ((x, y, cond) -> cond ? x : y);
        return c.from(a, b, condition);
    }

    public static int chooseBetween(int a, int b, boolean condition) {
        return condition ? a : b;
    }
 
    public static Double chooseBetween(Double a, Double b, boolean condition) {
        return condition ? a : b;
    }

}
