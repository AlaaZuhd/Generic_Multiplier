-----------------------------------------------------------
-- implemnation of the And gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs And
entity And2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity And2;					

-- defining and building the architecture of the 2-inputs And
architecture And2 of And2 is
begin	
	res <= In0 and In1;
end architecture And2;

-----------------------------------------------------------
-- implemnation of the OR gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs OR
entity Or2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity Or2;

-- defining and building the architecture of the 2-inputs OR
architecture Or2 of Or2	is
begin	
	 res <= In0 or In1; 
end architecture Or2;


-----------------------------------------------------------
-- implemnation of the XOR gate with two inputs. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2-inputs XOR
entity Xor2 is
	port(In0, In1: in std_logic;
		res: out std_logic);
end entity Xor2;

-- defining and building the architecture of the 2-inputs XOR
architecture Xor2 of Xor2 is
begin	
	 res <= In0 xor In1; 
end architecture Xor2;


-----------------------------------------------------------
-- Structural implementation of the one bit adder.

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 1-bit adder
entity One_Bit_Adder is
	port(In0, In1, Cin: in std_logic;
		Sum, Cout: out std_logic);
	--	in0 and in1 are the inputs to be added, and Cin is the carry in. 
	-- sum is the container of the addition result, and cout is for the carry out of the addition. 
end entity One_Bit_Adder;			 


-- defining and building the architecture of the 1-bit adder
architecture structural_One_Bit_Adder of One_Bit_Adder is 
signal s0, s1, s2: std_logic;
begin	
	
	Xor_In0_In1: entity work.Xor2(Xor2) port map (In0, In1, s0); 	
	Xor_S0_Cin: entity work.Xor2(Xor2) port map (s0, Cin, Sum);
	And_In0_In1: entity work.And2(And2) port map (In0, In1, s1);
	And_S0_Cin: entity work.And2(And2) port map (s0, Cin, s2);
	Or_S1_S2: entity work.Or2(Or2) port map (S1, S2, Cout); 
	-- this is eqivelant to => cout & Sum <= in0 + in1 + Cin. 
	
end architecture structural_One_Bit_Adder;


-----------------------------------------------------------
-- structural implementation of n-bit adder using generic parameter 'n' and One_Bit_Adder entity  
library ieee;
use ieee.std_logic_1164.all; 


-- defining the entity of the n-bit adder
ENTITY n_bit_adder IS
  GENERIC ( n: positive ); -- n -> generic parameter
  PORT (X, Y: IN STD_LOGIC_VECTOR(n-1 DOWNTO 0);
        Cin:  IN STD_LOGIC;
	    Sum: OUT STD_LOGIC_VECTOR(n-1 DOWNTO 0);
	    Cout: OUT STD_LOGIC); -- finish declaring the ports of the entity
		-- X and Y are the inputs to be added together. 
		-- cin is the carry in. 
		-- Sum is the register for the resulted sum.
		-- Cout is the carry out of the addition.
END ENTITY n_bit_adder;

-- internal functional of the n-bit adder 
architecture structural_N_Bit_Adder of N_Bit_Adder is 
signal Carry: std_logic_vector(n downto 0); -- carry signal to use in a the carry (in or out) for each one-bit-adder
begin	
	
	Carry(0) <= Cin; -- the first element in carry is the cin
	-- the last element in carry represents the carry out (final carry)resulted from the addition of the last bit
	Cout <= Carry(n);  
	
	-- add every two bits of the n-bit numbers X(i) and Y(i)
	-- and using the previous geenrated carry as the carry in for each current level of addition.
	generateLoop1: FOR i IN 0 TO n-1 GENERATE  
	     One_Bit_Adder_GateI:  ENTITY work.One_Bit_Adder(structural_One_Bit_Adder) PORT MAP (X(i),Y(i),Carry(i),Sum(i),Carry(i+1));
     END GENERATE generateLoop1;

end architecture structural_N_Bit_Adder;




-----------------------------------------------------------
-- implemnation of the 2X1MUX circuit. 

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the 2X1MUX   
entity MUX2X1 is	   
	generic (n,m: positive); 
	port(selection: std_logic;
	    Input0, Input1: in std_logic_vector(n+m downto 0);
		output: out std_logic_vector(n+m downto 0));  
	-- the first signal "selection" is the selection line of the mux, 
	-- input0 and input1 is two signals that the mux will choose one of them based on "selection" value, each of them are m+n bit
	-- output is the result of the max which is either input0 or input1, and it's also m+n bits number. 
end entity MUX2X1;					

-- defining and building the architecture of the 2X1MUX 
architecture MUX2X1 of MUX2X1 is
signal out1, out2: std_logic_vector(n+m downto 0);
begin	
	output <= input0 when selection ='0' else input1; 	
	-- if selection = 0, then the output will take the first signal "input0", else it will take the second signal "input1". 
end architecture MUX2X1;



-----------------------------------------------------------
-- implemnation of a special model for the right shift, such that it's euivelent to the usuall right-shift with an additional 
-- features, such that the new left-most bit will be equal to new_left_most_bit signal.  

-- Defining needed Libraries
library ieee;
use ieee.std_logic_1164.all; 

-- defining the entity of the special_right_shift. 
entity special_right_shift is	   
	generic (K: positive); 
	port(new_left_most_bit: std_logic;
	    Input: in std_logic_vector(K-1 downto 0);
		output: out std_logic_vector(K-1 downto 0));
end entity special_right_shift;					

-- defining and building the architecture of the special_right_shift.
architecture special_right_shift of special_right_shift is
begin	
	--output(K-1) <= new_left_most_bit; -- the left most bit take the value of new_left_most_bit signal 
--	
--	-- the work of this loop is identical to output(K-2 downto 0) <= input(k-1 downto 1) 
--	generateForLoop: for i in 0 to K-2 generate 
--		begin						  
--			output(i) <= Input(i+1); 
--		end generate generateForLoop; 	
	output <= new_left_most_bit & input(k-1 downto 1); 	
	-- the final result will be identical to the following instruction in VHDL : output <= new_left_most_bit & input(k-1 downto 1) 
end architecture special_right_shift;



-----------------------------------------------------------
-- structural implementation of the generic multiplier (K-bits*J-bits) 
-- using generic parameters 'k&J' with two differnet algorithms.  

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

-- defining the entity of the generic-multiplier.
entity Generic_Multiplier is 
	
	Generic (J, k: positive); -- geenric parameters where j and k represent the number of bits in X and Y respectively. 
	port (X: in std_logic_vector(j-1 downto 0);
		 Y: in std_logic_vector(k-1 downto 0);
		 res: out std_logic_vector((k+j-1) downto 0)); -- defining the port of the generic multiplier entity. 
		-- X is the first multiplicand, and Y is the multiplier. 
		-- res is the reuslt of multiplying X by Y. 
		 
end entity Generic_Multiplier;	 

-- implemenation of the generic multiplier using the first algorithm (Parallel Binary multiplication method),
-- which is depend mainly on the usage of the n-bit adder for k-1 stages. 
-- the inputs of each n-bit adder will be the and of Y(i) with X and the resulted cout&sum(n-1 downto1) from the result of n-bit-adder from the previous stage
-- expect for the first stage the first and the second operand will be the and of Y(0) with X & the and of Y(1) with X.
 
architecture Structural_Generic_Multiplier_Algorithm_NO_1 of Generic_Multiplier	is

Type array_1 is array (0 to k) of std_logic_vector(j-1 downto 0); -- array_1 is specified for registers temp_in_1, temp_in_2 and temp_out that used in the addition operation 
Type array_2 is array (0 to k-1) of std_logic; -- -- array_1 is specified for registers of kind Cout (the carry out of each addition operation)

signal  temp_out, temp_in_1, temp_in_2: array_1 :=(others=> (others=>'0'));
signal Cout: array_2;
signal temp1, temp2: std_logic_vector(j-1 downto 0);   
signal zeroBit: std_logic:='0';    

begin	
	-- generate the first input for the addition process (which is the and between bits of X and Y(0))
	gen1: for n in 0 to j-1 Generate 
	begin		  
		andJ101: ENTITY work.And2(And2) PORT MAP (X(n), Y(0), temp1(n)); 
    end generate ; 
	res(0) <= temp1(0); -- the right most bit in the result of the and between X and Y(0) is given to the firs bit in the final result.       
	ShifterI1: entity work.special_right_shift(special_right_shift) generic map (j) port map (zeroBit, temp1, temp_in_1(0));-- the first input to the adder is the right shift of the result of the and.
	-- loop k-1 times
	gen2: FOR i IN 1 TO k-1 GENERATE
  	BEGIN  
		-- preparing the second input for the n-bit adder which is the and between bits of X and Y(i+1). 
		gen2I: for L in 0 to j-1 Generate 
		begin		   
			andJ121: entity work.And2(And2) port map (X(L), Y(i), temp_in_2(i-1)(L)); 
		end generate gen2I; 	 
		-- after the inputs to the n-bit adder being ready, the addition process come to take a place, and here we use n-bit addeer such that n=j.
		AdderI: entity work.N_Bit_Adder(structural_N_Bit_Adder) generic map (j) 
																port map (temp_in_1(i-1), temp_in_2(i-1), '0', temp_out(i-1), Cout(i-1));	
		res(i) <= temp_out(i-1)(0); -- added a new bit (first bit in the output of the adder) to the final result.  	
		-- temp_in_1(i) <= Cout(i-1) & temp_out(i-1)(j-1 downto 1);	this line represented through the below line model
		ShifterI1: entity work.special_right_shift(special_right_shift) generic map (j) port map (Cout(i-1), temp_out(i-1), temp_in_1(i));-- update the value of the the next first input to the n-bit adder through the special right shift model
  	END GENERATE;  
	if_K_Is_1:
	if(k=1) generate
	begin
 		res <= '0' & temp1; 
	end generate if_K_Is_1;
	if_K_Is_Not_1:
	if(k/=1) generate 
	begin					   
		res(k+j-1 downto k-1) <= Cout(k-2) & temp_out(k-2); -- after the last addition operation, update the value of the last K-bits.
		--res(k+j-1 downto k-1) <= '0' & temp_out(k-2); -- the introduced error. 
	end generate if_K_Is_Not_1;
end architecture Structural_Generic_Multiplier_Algorithm_NO_1;





-- implemenation of the generic multiplier using the second algorithm (Add and shift method),
-- which happens on many levels (j levels), and the output for each level is either the shift for C&A$Q to the right 
-- or the addition of Y and A, and then shift C&A&Q to the right. 													
-- depending on the value of Q[i]. 
-- where C is a one-bit register, A is a j-bit register and Q is a j-bit register. 
-- After finishing all the iteration the final result will be held in A & Q respectively. 
  
architecture Structural_Generic_Multiplier_Algorithm_NO_2 of Generic_Multiplier	is
-- declaring 4 types of Arrays, each is specified for certain registers. 
Type array_1 is array (0 to j) of std_logic_vector(j-1 downto 0); -- array_1 is specified for registers of Kind Q 		   
Type array_2 is array (0 to j) of std_logic_vector(k-1 downto 0); -- array_2 is specified for registers of kind A
Type array_3 is array (0 to j) of std_logic;					  -- array_3 is specified for registers of kind C
Type array_4 is array (0 to j) of std_logic_vector(k+j downto 0); -- array_4 is specified for temp_in_1, temp_in_2 and temp_out that's specified for the mux operation. 

signal Q, Q_Shift, Q_Addition_Shift: array_1 :=(others=> (others=>'0')); 
-- Q contains the final reuslt of Q register after each iteration (which is eaither Q_Shift or Q_Addition_Shift). 
-- Q_shift contains the result of Q after each iteration, by considering the current action is shidt only
-- Q_Addition_shift contains the reuslt of Q after each iteration, by considering the current action is Addition then shift. 

signal A, A_shift, A_Add, A_Addition_Shift: array_2 :=(others=> (others=>'0'));
-- A contains the final reuslt of A register after each iteration (which is eaither A_Shift or A_Addition_Shift). 
-- A_shift contains the result of A after each iteration, by considering the current action is shidt only
-- A_Add contsins the reuslt of A after eahc iteration, by considering the Addition process for Addition and shift action. 
-- A_Addition_shift contains the reuslt of A after each iteration, by considering the current action is Addition then shift. 

signal C, C_Shift, C_Add, C_Addition_Shift: array_3:= (others=> '0'); 	 
-- C contains the final reuslt of C register after each iteration (which is eaither C_Shift or C_Addition_Shift). 
-- C_shift contains the result of C after each iteration, by considering the current action is shidt only
-- C_Add contsins the reuslt of C after eahc iteration, by considering the Addition process for Addition and shift action. 
-- C_Addition_shift contains the reuslt of C after each iteration, by considering the current action is Addition then shift. 

signal temp_in_1, temp_in_2, temp_out: array_4 := (others=> (others=> '0'));	 
-- temp_in_1 is a temporarily register for the first input of the 2X1mux (C_Shift(i) & A_Shift(i) & Q_Shift(i)).
-- temp_in_2 is a temporarily register for the second input of the 2X1mux (C_Addition_Shift(i) & A_Addition_Shift(i) & Q_Addition_Shift(i)).							   
-- temp_out is a temporarily register for the output of the 2X1mux (C(i+1)&A(i+1)&Q(i+1)).
begin	
	Q(0) <= X; -- initalize Q(0) to be X, while the initalization of the rest registers done above.  
	gen1: for i in 0 to j-1 generate -- to loop through the bits of Q register (j times).
		begin		 
			-- stage1_1: First consider the action for the of Q(i)(0) is shift only 
			C_shift(i) <= '0';  -- do the right shift for C register and store the reuslt is C_shift.  
			ShifterI1: entity work.special_right_shift(special_right_shift) generic map (k) port map (C(i), A(i), A_Shift(i));-- do the right shift for A register and store the reuslt is A_shift, also take of C(i) as the one that will be added to the left of A_shift. 
			ShifterI2: entity work.special_right_shift(special_right_shift) generic map (j) port map (A(i)(0), Q(i), Q_Shift(i)); -- do the right shift for Q register and store the reuslt is AQ_shift, also take of A(i)(0) as the one that will be added to the left of Q_shift. 
			
			-- stage1_2: Second consider the action for the bit of Q(i)(0) is Add then shift.  																	  
			AdderI: entity work.N_Bit_Adder(Structural_N_Bit_Adder) generic map(k) port map(A(i), Y, C(i), A_Add(i), C_Add(i));	-- do the addition between A(i) and Y. 
			-- after the addition do the same as in stage1_1, which is the shift of C&A_Add&Q_Add.
			C_Addition_shift(i) <= '0';  
			ShifterI3: entity work.special_right_shift(special_right_shift) generic map (k) port map (C_Add(i), A_Add(i), A_Addition_Shift(i));
			ShifterI4: entity work.special_right_shift(special_right_shift) generic map (j) port map (A_Add(i)(0), Q(i), Q_Addition_Shift(i));
			
			-- stage2 : Now the turn come to select one of the results of the above two sub_stages, by using a 2X1MUX with a selction = Q(i)(0). 	
			temp_in_1(i) <= C_Shift(i) & A_Shift(i) & Q_Shift(i);
			temp_in_2(i) <= C_Addition_Shift(i) & A_Addition_Shift(i) & Q_Addition_Shift(i);
			Mux2X1: entity work.MUX2X1(MUX2X1) generic map(j,k) port map(Q(i)(0), temp_in_1(i), temp_in_2(i), temp_out(i));
			-- finally update the values of C(i+1), A(i+1) and Q(i+1) based on temp_out
			C(i+1) <= temp_out(i)(k+j);  
			A(i+1) <= temp_out(i)(k+j-1 downto j); 
			Q(i+1) <= temp_out(i)(j-1 downto 0);
		end generate gen1;
	res <= A(j) & Q(j); -- the final result is held in A(j) & Q(j).
	--res <= A(j-1) & Q(j-1); -- the introduced error
end architecture Structural_Generic_Multiplier_Algorithm_NO_2; 



-- result analyszer 
library ieee;
use ieee.std_logic_1164.all; 

-- declaring the entity of the result analyzer. 
entity result_analyzer is
	generic (k,j : positive); -- using two generic parameters k and j 
	port(clock: in std_logic;
		ExpectedResult, ActuallResult: in std_logic_vector (k+j-1 downto 0)
		);															   
end entity result_analyzer;	 

architecture result_analyzer OF result_analyzer is
begin
    process(clock)
    begin
      if rising_edge(clock) then
        -- Check whether Generic_multiplier output matches expectation which wa getten from using * oerator (X*Y).  
        assert   ExpectedResult =  ActuallResult -- if ExpectedResult != ActuallResult then assert a report of warning severity
        report   "Multiplier output is incorrect" -- in case of an error this meesage will be displayed into the consle
        severity WARNING;
      end if;
    end process;
end architecture result_analyzer;


-- test generater
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

-- entity declartion of test generator 
entity test_generator is   
	generic (k,j: positive); -- two generic parameters K and J. 
	port(clock: in std_logic;
		TestIn1:  out std_logic_vector(k-1 downto 0);
		TestIn2: out std_logic_vector(j-1 downto 0);
		ExpectedResult: out std_logic_vector(j+k-1 downto 0)
	);	 
	-- TestIn1 and TestIn2 are the two inputs for the generic multiplier. 
	-- ExpectedResult is the result of the multiplication of the two numbers. 
end entity test_generator;	 

-- description of the test generator 	
-- this test generator loops through all possible cases of the two inputs for the multiplier (based on their number of bits)
-- and generat the test inputs for each possible case, and the expected result of the multiplication. 
architecture test_generator of test_generator is  	
begin
	process
		begin
			for I in 0 to 2**k-1 loop -- this loop to generate all possible value of the first number (which have a range from 0 to (2^k)-1).
		    	for L in 0 to 2**j-1 loop -- this loop to generate all possible value of the second number (which have a range from 0 to (2^j)-1).
		      		-- set the inputs to the multiplier, by converting I and L into a bonary std_logic vector of K and J bits respectively. 
		      		TestIn1 <= conv_std_logic_vector(i,k); 
		      		TestIn2 <= CONV_STD_LOGIC_VECTOR(L,j);
		      		-- Calculate what the output of the multiplier should be
		      		ExpectedResult <= CONV_STD_LOGIC_VECTOR(I*L,k+j);
		      		-- Wait until multiplier output has settled
		      		wait until rising_edge(clock);
		    	end loop;
		  	end loop;
		wait;
	end process;
   
end architecture test_generator;


-- test bench for each algorithm 
library ieee;
use ieee.std_logic_1164.all; 
-- entity of test bench 
entity test_bench is
end entity test_bench;	 

-- test bench architecture of the first algorithm. 
architecture test_bench_1 of test_bench is
signal clock: std_logic:='0'; -- clock signal used here for running the test_generator at each rising edge. 
signal TestIn1: std_logic_vector(6 downto 0); -- the first input of the multiplier 
signal TestIn2: std_logic_vector(3 downto 0); -- the seocnd input of the multiplier   
signal ExpectedResult, ActuallResult: std_logic_vector(10 downto 0); -- the escpected and actuall result of the multiplier
begin
	
 -- Place one instance of test generator (from this model we can generate the test inputs and the expected result)
 	TG: entity work.test_generator(test_generator) generic map (7,4) 
		                                          port map(clock, TestIn1, TestIn2, ExpectedResult);
 -- Place one instance of the Unit Under Test  which is the generic multiplier fot he first algorithm   
 ---UUT: entity work.m(mm) generic map (7,4) port map(TestIn1, TestIn2, ActuallResult);	
	 -- Place one instance of the Unit Under Test which is the generic multiplier fot he second algorithm 
    UUT: entity work.Generic_Multiplier(Structural_Generic_Multiplier_Algorithm_NO_1) generic map (7,4) 
	                                                                                  port map(TestIn1, TestIn2, ActuallResult);
	 
 -- Place one instance of result analyzer (the one that will check if the result obtained from the built model is as the expected result or not)
    RA: entity work.result_analyzer(result_analyzer) generic map (7,4) 
													 port map(clock, ExpectedResult, ActuallResult); 	
	clock <= not clock after 5 ns; -- flip the clock after each 5 ns. 
	
end architecture test_bench_1;	

-- test bench architecture of the first algorithm. 
architecture test_bench_2 of test_bench is
signal clock: std_logic:='0'; -- clock signal used here for running the test_generator at each rising edge.
signal TestIn1: std_logic_vector(6 downto 0); -- the first input of the multiplier 
signal TestIn2: std_logic_vector(3 downto 0);  -- the second input of the multiplier  
signal ExpectedResult, ActuallResult: std_logic_vector(10 downto 0); -- the escpected and actuall result of the multiplier
begin
	
 -- Place one instance of test generator (from this model we can generate the test inputs and the expected result)
 	TG: entity work.test_generator(test_generator) generic map (7,4) 
		                                          port map(clock, TestIn1, TestIn2, ExpectedResult);
 -- Place one instance of the Unit Under Test which is the generic multiplier fot he second algorithm 
    UUT: entity work.Generic_Multiplier(Structural_Generic_Multiplier_Algorithm_NO_2) generic map (7,4)  
	                                                                                  port map(TestIn1, TestIn2, ActuallResult);  

 -- Place one instance of result analyzer (the one that will check if the result obtained from the built model is as the expected result or not)
    RA: entity work.result_analyzer(result_analyzer) generic map (7,4) 
													 port map(clock, ExpectedResult, ActuallResult); 	
	clock <= not clock after 5 ns; -- flip the clock after each 5 ns.  
	
end architecture test_bench_2; 

-----------------------------------------------------------

