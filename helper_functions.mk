TO_UPPER_LETTERS := \
  a,A b,B c,C d,D e,E f,F g,G h,H \
  i,I j,J k,K l,L m,M n,N o,O p,P \
  q,Q r,R s,S t,T u,U v,V w,W x,X \
  y,Y z,Z

TO_LOWER_LETTERS := \
  A,a B,b C,c D,d E,e F,f G,g H,h \
  I,i J,j K,k L,l M,m N,n O,o P,p \
  Q,q R,r S,s T,t U,u V,v W,w X,x \
  Y,y Z,z

 to-x-aux = \
   $(if $2, \
     $(eval TO_UPPER_AUX_RESULT = $$(subst $(firstword $2), \
       $(call to-x-aux,$1,$(wordlist 2,$(words $2),$2))))$(TO_UPPER_AUX_RESULT), \
     $1 \
     )

to-upper = $(strip $(call to-x-aux,$1,$(TO_UPPER_LETTERS)))
to-lower = $(strip $(call to-x-aux,$1,$(TO_LOWER_LETTERS)))

# Example usage: 
#
# TEST_WORD := Hello, world!
# $(info $$(call to-upper,'$(TEST_WORD)') -> '$(call to-upper,$(TEST_WORD))')
