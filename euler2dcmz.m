%eul defined in "n": rotate "n" to obtain "b"
%result: Cbn (from b to n) * [0 0 1]'
function dcmz = euler2dcmz(eul)
dcmz = zeros(3,size(eul,2));
for n=1:size(eul,2)
    Cbn=diag([-1 -1 1])*euler2dcm_v000(eul(:,n));
    dcmz(:,n)=(Cbn*[0 0 -1]');
end

