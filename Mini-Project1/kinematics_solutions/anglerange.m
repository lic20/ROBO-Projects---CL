function q1=anglerange(q)

    q1=(abs(q)>pi).*(q-sign(q-pi)*2*pi)+(abs(q)<=pi).*q;

end