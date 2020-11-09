clear all;close all;clc;
tag_type='tagStandard52h13';
tag_short_type='52h13';
tagsize=10;
offset=2;
rescaling=20;
max_ntag=1000;

im=imread(sprintf('%s_mosaic.png',tag_type));

nr=floor((size(im,1)+1)/(tagsize+1));
nc=floor((size(im,2)+1)/(tagsize+1));


for ir=1:nr
    for ic=1:nc
        ntag=(ir-1)*nr+(ic-1);
        if ntag>=max_ntag
            break;
        end
        tag=ones(tagsize+offset*2,tagsize+offset*2);
        tag(offset+(1:tagsize),offset+(1:tagsize))=im((ir-1)*(tagsize+1)+(1:tagsize),(ic-1)*(tagsize+1)+(1:tagsize));
        
        tag_color = cat(3, tag, tag, tag);
        tag_color(1,:,2:3)=0;
        tag_color(end,:,1:2)=0;
        tag_color(:,1,[1 3])=0;
        tag_color(:,end,3)=0;
        tag_color([1 end],[1 end],:)=0;
        
        tag_rescaled_size=2+(tagsize+offset*2)*rescaling;
        tag_color_resize=ones(tag_rescaled_size,tag_rescaled_size,3);
        
        tag_color_resize(2:end-1,2:end-1,:)=imresize(tag_color,rescaling,'nearest');
        tag_name=sprintf('%s_%05d.png',tag_type,ntag);
        
        top_string=sprintf('ID%4d type %s +Y',ntag,tag_short_type);
        right_string=sprintf('+X',tag_short_type,ntag);
        tag_text=insertText(tag_color_resize,[rescaling+1 1;tag_rescaled_size-rescaling-1 0.5*(tag_rescaled_size-rescaling)-1],{top_string,right_string},'FontSize',14,'BoxOpacity',0.0,'Font','Arial');
        %imshow(tag_text)
        
        imwrite(tag_text,tag_name);
    end
end