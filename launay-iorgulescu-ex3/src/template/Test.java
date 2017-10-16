package template;

import java.util.Arrays;
import java.util.HashSet;

public class Test {

	public int[] in;
	
	
	public static void main(String[] args) {
		int[] a = new int[] {1,2,3};
		int[] b = new int[] {1,2,3};
		
		Test c = new Test();
		Test d = new Test();
		
		HashSet<Test> h = new HashSet<Test>();
		h.add(c);
		System.out.println(a.hashCode()+" "+b.hashCode()+" "+c.hashCode()+" "+d.hashCode());
		System.out.println(c.equals(d));
		System.out.println(c.hashCode()==d.hashCode());
		System.out.println(h.contains(d));
	}
	
		
	public boolean equals(Object obj) {
		return true;
	}

		
		public int hashCode() {
			return 42;
		}
	
	
		public Test() {
		}
	
		public Test(int[] b) {
			in=b;
		}
	}
	
