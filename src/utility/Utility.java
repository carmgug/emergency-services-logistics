package utility;

import esl.Node;

import java.time.Period;
import java.util.HashMap;

public final class Utility {


    public static int howManyTimesContainsThisPredicate(String predicate_name, int[] state, HashMap<Integer, Predicate> translator){
        int count=0;
        for(int x:state){
            Predicate p=translator.get(x);
            if(predicate_name.equals(p.getName())) count++;
        }
        return count;
    }


}
