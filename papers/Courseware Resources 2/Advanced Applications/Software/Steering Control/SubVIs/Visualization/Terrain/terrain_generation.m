function terrain_generation(generate_file_output, terrain_file, terrain_shadow_file, road_file, fence_file, map_file, path_to_scene_file, shadow_map_file)

if (nargin == 1)
   if generate_file_output == true
      disp('If generate_file_output is true, the output files must be specified.');
      return;
   end
end

close all;
clc;

global gridxy;
global sizexy;
global sizez;

gridxy = 200;
sizexy = 2000;
sizez = 150;

surface = create_surfaces();
road = create_road();
fence = create_fence();
trees = create_trees();

if (generate_file_output)
%      generate_x3d_terrain(surface, terrain_file, terrain_shadow_file);
%      generate_x3d_road(road, road_file);    
%      generate_x3d_fence(fence, fence_file);
%      generate_road_map(road, map_file, 512);
     add_trees_to_scene(trees, path_to_scene_file);
     generate_tree_shadow_map(trees, shadow_map_file, 4096);
end


% rotate surface


[m,n] = size(surface);
surface_rotate = zeros(n,m);

for x = 1:m
    for y = 1:n
        surface_rotate(y,x) = surface(x,y);
    end
end

figure('Position', [300 550 1000 400])
subplot(1,2,1);
mesh(linspace(0, sizexy-gridxy/sizexy, gridxy), linspace(0, sizexy-gridxy/sizexy, gridxy), surface_rotate)
hold on;

plot3(road(1,:), road(2,:)+1, road(3,:)+5);
plot3(road(4,:), road(5,:)+1, road(6,:)+5);

plot3(fence(1,:), fence(2,:)+1, fence(3,:)+5, 'r');
plot3(fence(4,:), fence(5,:)+1, fence(6,:)+5, 'r');
hold off;
xlabel('x')
ylabel('y')

subplot(1,2,2);

mesh(linspace(0, sizexy-gridxy/sizexy, gridxy), linspace(0, sizexy-gridxy/sizexy, gridxy), surface_rotate)
hold on;

plot3(road(1,:), road(2,:)+1, road(3,:)+5);
plot3(road(4,:), road(5,:)+1, road(6,:)+5, 'g');


plot3(fence(1,:), fence(2,:)+1, fence(3,:)+5, 'r');
plot3(fence(4,:), fence(5,:)+1, fence(6,:)+5, 'r');
plot3(trees(1,:), trees(2,:)+1, trees(3,:)+5, '+g');


xlabel('x')
ylabel('y')
hold off;




view(0,90)
axis square

%%
function [trees] = create_trees()

global sizexy;

tree_count = 1000;


rand('seed', 0);

trees = zeros(3, tree_count);

for count = 1:tree_count
    
    recalculate = 1;
    retry = 0;
    
    while (recalculate)
        retry = retry + 1;
        if retry == 100
            error('Could not find a place to put at tree');
        end
    
        trees(1:2, count) = [rand() rand()]'*sizexy;
        
        %check against fence equations
        
        cx = 900;  %must be the same as the track cx
        cy = 1050; %must be the same as the track cy
        
        theta = atan2(trees(2,count) - cy, trees(1,count) - cx);
        if (theta < 0)
            theta = theta + 2*pi;
        end       
        
        tree_radius = sqrt( (trees(1,count)-cx)^2 + (trees(2,count)-cy)^2 );
        
        Fr = (200 + 100*sin(-2*theta));
        if (tree_radius < Fr)
            recalculate = 0;
        end
    
        Fr = (400 + 100*sin(-2*theta)  );        
        if ((tree_radius > Fr) && (tree_radius < 600))
            recalculate = 0;
        end 
        
        %also check that it is not within the boundaries of another tree
        if (recalculate == 0)
            if count > 1
                for count_check = 1:(count-1)
                    if (sqrt( (trees(1,count)-trees(1,count_check))^2 + (trees(2,count)-trees(2,count_check))^2  ) < 12)
                        recalculate = 1;
                    end

                end
            end
        end
    end
        
%     trees(1,count) = 1500 + count*10;
%     trees(2,count) = 1000;
% 
    trees(3, count) = calc_surface_z(trees(1,count), trees(2,count));
    
end


%%
function [road] = create_road()


r_center = 300;
road_width = 7.5;

r = r_center - road_width/2;
cx = 900;
cy = 1050;

high_freq_center = 2*pi*3/4;
high_freq_region_width = 1;

polar_pos = linspace(0,2*pi,1000);


road = zeros(6, size(polar_pos, 2));
index = 1;

for theta = polar_pos
    %road(1, index) = cx + (r + 50*sin(2*theta)             + 100*exp((-(theta-high_freq_center).^2)./(2*high_freq_region_width^2)).*sin(15*theta).*sin(4*theta) ).*cos(theta);
    %road(2, index) = cy + (r + 50*sin(4*theta).*sin(theta) + 100*exp((-(theta-high_freq_center).^2)./(2*high_freq_region_width^2)).*sin(15*theta).*sin(4*theta) ).*sin(theta);
    road(4, index) = cx + (r + 100*sin(-2*theta) + 25*sin(4*theta) + 100*exp((-(theta-high_freq_center).^2)./(2*high_freq_region_width^2)).*sin(10*theta).*sin(4*theta)).*cos(theta);
    road(5, index) = cy + (r + 100*sin(-2*theta)                   + 100*exp((-(theta-high_freq_center).^2)./(2*high_freq_region_width^2)).*sin(10*theta).*sin(4*theta)).*sin(theta);
    
    road(6, index) = calc_surface_z(road(4, index),road(5, index)); 
    
    if (index == 1)
        road(1, index) = 0;
        road(2, index) = 0; 
    else       
        [road(1, index) road(2, index)] = road_offset(road(4, index-1), road(5, index-1), road(4, index), road(5, index), road_width);
    end
    road(3, index) = calc_surface_z(road(1, index),road(2, index)); 
    
    index = index + 1;
    
end

% now need to correct the first point
[road(1, 1) road(2, 1)] = road_offset(road(4, 2), road(5, 2), road(4, 1), road(5, 1), -road_width);
road(3, 1) = calc_surface_z(road(1, 1),road(2, 1)); 

disp(sprintf('Car initial position = [%f %f %f]\n', road(1, 1), road(2, 1), road(3, 1)));
disp(sprintf('Car initial theta = %f\n', atan2(road(2, 2)-road(2,1), road(1, 2)-road(1,1))));


%%
function map = generate_road_map(road, map_file, map_size)

global sizexy;

dot = [    0    0    0    0    0    1    2    2    2    2    1    0    0    0    0    0
    0    0    0    1    3    6   10   12   12   10    6    3    1    0    0    0
    0    0    2    5   12   20   28   32   32   28   20   12    5    2    0    0
    0    1    6   14   28   42   56   64   64   56   42   28   14    6    1    0
    1    4   13   28   50   74   96  108  108   96   74   50   28   13    4    1
    2    9   24   47   77  114  142  157  157  142  114   77   47   24    9    2
    3   14   33   62  105  150  184  203  203  184  150  105   62   33   14    3
    4   17   41   77  121  168  209  232  232  209  168  121   77   41   17    4
    4   17   41   77  125  176  217  242  242  217  176  125   77   41   17    4
    4   16   39   74  116  162  201  223  223  201  162  116   74   39   16    4
    2   12   29   55   95  139  170  187  187  170  139   95   55   29   12    2
    1    7   20   41   66   99  124  138  138  124   99   66   41   20    7    1
    0    3    9   22   41   60   80   91   91   80   60   41   22    9    3    0
    0    1    4   10   21   33   43   50   50   43   33   21   10    4    1    0
    0    0    1    3    7   14   20   22   22   20   14    7    3    1    0    0
    0    0    0    0    2    3    5    7    7    5    3    2    0    0    0    0]/255;

dot_center = floor(size(dot))';

map = ones(map_size,map_size,3);
alpha = zeros(map_size, map_size);
for count = 1:length(road(1,:))

    %alpha(x, y) = 1;
    %map(map_size - floor(road(2,count)/gridy*map_size)+1, floor(road(1,count)/gridx*map_size)+1,  :) = [0 0 0];
    
    current_pos = [map_size - floor(  (road(5,count)/sizexy*map_size + road(2,count)/sizexy*map_size)/2 )
                   floor(  (road(4,count)/sizexy*map_size + road(1,count)/sizexy*map_size)/2 )] - dot_center;
    
    for x = 1:size(dot,1)
        for y = 1:size(dot,2)
            alpha(current_pos(1)+x, current_pos(2)+y) = alpha(current_pos(1)+x, current_pos(2)+y) + dot(x,y);
            
            if (alpha(current_pos(1)+x, current_pos(2)+y,1) > 1.0)
                alpha(current_pos(1)+x, current_pos(2)+y,:) = 1;
            end
        end
    end

end

alpha = alpha*0.8;
   
imwrite(map, map_file, 'png', 'Alpha', alpha);



function map = generate_tree_shadow_map(trees, map_file, map_size)

global sizexy;

dot = imread('single_tree_shadow.png', 'png');
alpha_dot = zeros(size(dot,1), size(dot,2));

alpha_dot = dot(:,:,1);
alpha_dot = double((255.0-double(alpha_dot))./255.0);



map = zeros(map_size,map_size,3);
alpha = zeros(map_size, map_size);
for count = 1:size(trees(1,:),2)

    current_pos = [map_size - floor(  trees(2,count)/sizexy*map_size) 
                   floor(  trees(1,count)/sizexy*map_size)] + [-130; -20];
    
    for x = 1:size(alpha_dot,1)
        for y = 1:size(alpha_dot,2)
            alpha(current_pos(1)+x, current_pos(2)+y) = alpha(current_pos(1)+x, current_pos(2)+y) + alpha_dot(x,y);
            
            if (alpha(current_pos(1)+x, current_pos(2)+y,1) > 1.0)
                alpha(current_pos(1)+x, current_pos(2)+y,:) = 1;
            end
        end
    end

end

alpha = alpha*0.8;
   
imwrite(map, map_file, 'png', 'Alpha', alpha);

%%
function [x_off y_off] = road_offset(x0, y0, x1, y1, offset)

angle = atan2(y1-y0, x1-x0);
angle = angle - pi/2;

x_off = x1 - offset*cos(angle);
y_off = y1 - offset*sin(angle);



%%
function [road] = create_fence()


cx = 900;  %must be the same as the track cx
cy = 1050; %must be the same as the track cy

polar_pos = linspace(0,2*pi,400);


road = zeros(6, size(polar_pos, 2));
index = 1;

% Note, the fence design must be:
%
%  x = F(theta)*cos(theta), 
%  y = F(theta)*sin(theta
%
% Effectively, it must be a circle with varying radius, so F must be the
% same for both x and y.

for theta = polar_pos
    
    %Fr = (r_inner + 50*sin(4*theta) - 100*exp((-(theta - pi*5/3).^2)./(2* 0.35 ^2)) );
    Fr = (200 + 100*sin(-2*theta));
    
    road(1, index) = cx + Fr*cos(theta);
    road(2, index) = cy + Fr*sin(theta);
    road(3, index) = calc_surface_z(road(1, index),road(2, index));  
    
    Fr = (400 + 100*sin(-2*theta)  );
    road(4, index) = cx + Fr*cos(theta);
    road(5, index) = cy + Fr*sin(theta);
    road(6, index) = calc_surface_z(road(4, index),road(5, index)); 
    
    index = index + 1;
    
end






%%
function [surface] = create_surfaces()

global gridxy;
global sizexy;

surface = zeros(gridxy,gridxy);

for i = 1:size(surface,1)
    for j = 1:size(surface,2)
       surface(i,j) = calc_surface_z((i-1)*sizexy/gridxy,(j-1)*sizexy/gridxy); 
    end
end

%%
function z = calc_surface_z(x, y)

global sizexy;
global sizez;

z = (sin(y*0.008 + 2)*sin(x*0.01)*cos(y*0.002) + cos(x*0.03)*sin(y*0.04)*0.1) + 1;

z = z*sigmoid(x, y, sizexy); 
z = z*0.5*sizez;

%%
function scale = sigmoid(x,y,max_xy)

%sigmoid parameters

b1 = max_xy*0.1;
b2 = max_xy - max_xy*0.1;
c = 0.01;

scalex = 1./(1+exp(-c.*(x-b1))) - 1./(1+exp(-c.*(x-b2)));
scaley = 1./(1+exp(-c.*(y-b1))) - 1./(1+exp(-c.*(y-b2)));

scale = scalex*scaley;



%%
function generate_x3d_terrain(surface, terrain_file, terrain_shadow_file)

global sizexy;

disp('Generating Terrain X3D File...');
disp('Generating Indices...');
[m,n] = size(surface);
indexed_face_set = zeros((m-1)*(n-1)*4*2, 1);
index_count = 1;

for y = 1:(n-1)
    for x = 1:(m-1)
        indexed_face_set(index_count + 0) = x-1 + n*(y-1);
        indexed_face_set(index_count + 1) = x-0 + n*(y-1);
        indexed_face_set(index_count + 2) = x-1 + n*(y-0);
        indexed_face_set(index_count + 3) = -1;

        indexed_face_set(index_count + 4) = x-0 + n*(y-1);
        indexed_face_set(index_count + 5) = x-0 + n*(y-0);
        indexed_face_set(index_count + 6) = x-1 + n*(y-0);
        indexed_face_set(index_count + 7) = -1;      

        index_count = index_count + 8;
    end 
end

disp('Generating Normals...');
surface_normals = ones(m, n, 3);

delta = 1/m*sizexy;


for y = 1:n
    for x = 1:m
        if (x == 1) || (x == m) || (y == 1) || (y == n)
            surface_normals(x, y, 1) = 0;
            surface_normals(x, y, 2) = 0;
            surface_normals(x, y, 3) = 1;
            
        else
            center_z = surface(x,y);
            
            vecN = [ 0 (delta)  (surface(x,y+1) - center_z)];
            vecE = [ (delta)  0 (surface(x+1,y) - center_z)];
            vecS = [ 0 (-delta) (surface(x,y-1) - center_z)];
            vecW = [ (-delta) 0 (surface(x-1,y) - center_z)];

            normal = cross(vecN, vecW) + cross(vecW, vecS) + cross(vecS, vecE) + cross(vecE, vecN);
            normal_component_total = norm(normal);

            surface_normals(x, y, 1) = normal(1)/normal_component_total;
            surface_normals(x, y, 2) = normal(2)/normal_component_total;
            surface_normals(x, y, 3) = normal(3)/normal_component_total;
        end        
    end
end

disp('Writing File Terrain...');

fid = fopen(terrain_file, 'w');

try
    fprintf(fid, '<?xml version="1.0" encoding="utf-8"?>\n');
    fprintf(fid, '<X3D profile="Immersive" version="3.0">\n');
    fprintf(fid, '	<Scene>\n');
    fprintf(fid, '		<Shape>\n');
    
    disp('-> Indices');
    fprintf(fid, '			<IndexedFaceSet coordIndex="');
    
    for count = 1:size(indexed_face_set, 1)
        fprintf(fid, '%d ', indexed_face_set(count,1));
    end
        
    fprintf(fid, '">\n');

    disp('-> Vertices');
    fprintf(fid, ' 				<Coordinate point="');
       
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f %2.3f ', (x-1)/m*sizexy, (y-1)/m*sizexy, surface(x,y));
        end 
    end  
    
    fprintf(fid, '"/>\n');
    
    

    disp('-> Normals');
    fprintf(fid, ' 				<Normal vector="');
       
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(x,y,1), surface_normals(x,y,2), surface_normals(x,y,3));
        end 
    end  
    
    fprintf(fid, '"/>\n');    
    
    
    disp('-> Texture Coordinates');
    fprintf(fid, ' 				<TextureCoordinate point="');
       
    texture_scale = 0.3;
    
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f ', x*texture_scale, y*texture_scale);
        end 
    end 
   
    fprintf(fid, '"/>\n');    
    

    fprintf(fid, '			</IndexedFaceSet>\n');
    fprintf(fid, '			<Appearance>\n');
    fprintf(fid, '				<Material diffuseColor="1.0 1.0 1.0"/>\n');
    fprintf(fid, '			</Appearance>\n');
    fprintf(fid, '		</Shape>\n');
    fprintf(fid, '		<Viewpoint description="View Object 0" orientation="1 0 0 0" position="0.000000 0.000000 2.500000"/>\n');
    fprintf(fid, '		<NavigationInfo type="EXAMINE"/>\n');
    fprintf(fid, '	</Scene>\n');
    fprintf(fid, '</X3D>\n');
catch
    fclose(fid);
    
    disp('Error caught.  Closed file');
end


fclose(fid);




disp('Writing File Shadow Terrain...');

fid = fopen(terrain_shadow_file, 'w');

try
    fprintf(fid, '<?xml version="1.0" encoding="utf-8"?>\n');
    fprintf(fid, '<X3D profile="Immersive" version="3.0">\n');
    fprintf(fid, '	<Scene>\n');
    fprintf(fid, '		<Shape>\n');
    
    disp('-> Indices');
    fprintf(fid, '			<IndexedFaceSet coordIndex="');
    
    for count = 1:size(indexed_face_set, 1)
        fprintf(fid, '%d ', indexed_face_set(count,1));
    end
        
    fprintf(fid, '">\n');

    disp('-> Vertices');
    fprintf(fid, ' 				<Coordinate point="');
       
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f %2.3f ', (x-1)/m*sizexy, (y-1)/m*sizexy, surface(x,y));
        end 
    end  
    
    fprintf(fid, '"/>\n');
    
    
    disp('-> Normals');
    fprintf(fid, ' 				<Normal vector="');
       
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(x,y,1), surface_normals(x,y,2), surface_normals(x,y,3));
        end 
    end  
    
    fprintf(fid, '"/>\n');        
    
    
    disp('-> Texture Coordinates');
    fprintf(fid, ' 				<TextureCoordinate point="');
       
    
    for y = 1:n
        for x = 1:m
            fprintf(fid, '%2.3f %2.3f ', (x-1)/(m-1), (y-1)/(n-1));
        end 
    end 
   
    fprintf(fid, '"/>\n');    
    

    fprintf(fid, '			</IndexedFaceSet>\n');
    fprintf(fid, '			<Appearance>\n');
    fprintf(fid, '				<Material diffuseColor="1.0 1.0 1.0"/>\n');
    fprintf(fid, '			</Appearance>\n');
    fprintf(fid, '		</Shape>\n');
    fprintf(fid, '		<Viewpoint description="View Object 0" orientation="1 0 0 0" position="0.000000 0.000000 2.500000"/>\n');
    fprintf(fid, '		<NavigationInfo type="EXAMINE"/>\n');
    fprintf(fid, '	</Scene>\n');
    fprintf(fid, '</X3D>\n');
catch
    fclose(fid);
    
    disp('Error caught.  Closed file');
end


fclose(fid);

disp(sprintf('File written.\n'));


%%
function generate_x3d_road(road, output_file)

disp('Generating Road X3D File...');
disp('Generating Indices...');
m = size(road,2);
indexed_face_set = zeros((m-1)*4*2*3, 1);
index_count = 1;

road_height = 1;

for x = 0:(m-2)
    for y = 0:2
        indexed_face_set(index_count + 0) = x*4 + y + 0;
        indexed_face_set(index_count + 1) = x*4 + y + 4;
        indexed_face_set(index_count + 2) = x*4 + y + 1;
        indexed_face_set(index_count + 3) = -1;

        indexed_face_set(index_count + 4) = x*4 + y + 4;
        indexed_face_set(index_count + 5) = x*4 + y + 5;
        indexed_face_set(index_count + 6) = x*4 + y + 1;
        indexed_face_set(index_count + 7) = -1;     

        index_count = index_count + 8;
    end
end 



disp('Generating Normals...');
surface_normals = ones(m, 6);

for count = 1:(m-1)
    
    vecE = [ road(1, count+1)-road(1, count)  road(2, count+1)-road(2, count)  road(3, count+1)-road(3, count)];
    vecS = [ road(4, count)-road(1, count)    road(5, count)-road(2, count)    road(6, count)-road(3, count)];


    normal = cross(vecS, vecE);
    normal_component_total = norm(normal);

    surface_normals(count, 1) = normal(1)/normal_component_total;
    surface_normals(count, 2) = normal(2)/normal_component_total;
    surface_normals(count, 3) = normal(3)/normal_component_total;
    
    surface_normals(count, 4:6) = norm(vecS);
     
end

surface_normals(m, 1) = surface_normals(1, 1);
surface_normals(m, 2) = surface_normals(1, 2);
surface_normals(m, 3) = surface_normals(1, 3);
surface_normals(m, 4) = surface_normals(1, 4);
surface_normals(m, 5) = surface_normals(1, 5);
surface_normals(m, 6) = surface_normals(1, 6);


disp('Writing File...');

fid = fopen(output_file, 'w');

try
    fprintf(fid, '<?xml version="1.0" encoding="utf-8"?>\n');
    fprintf(fid, '<X3D profile="Immersive" version="3.0">\n');
    fprintf(fid, '	<Scene>\n');
    fprintf(fid, '		<Shape>\n');
    
    disp('-> Indices');
    fprintf(fid, '			<IndexedFaceSet coordIndex="');
    
    for count = 1:size(indexed_face_set, 1)
        fprintf(fid, '%d ', indexed_face_set(count,1));
    end
        
    fprintf(fid, '">\n');

    disp('-> Vertices');
    fprintf(fid, ' 				<Coordinate point="');
       
    for count = 1:m
        fprintf(fid, '%2.3f %2.3f %2.3f ', road(4,count), road(5,count), road(6,count) - road_height);
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', road(4,count), road(5,count), road(6,count));
        fprintf(fid, '%2.3f %2.3f %2.3f ', road(1,count), road(2,count), road(3,count));
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', road(1,count), road(2,count), road(3,count) - road_height);
    end  
    
    fprintf(fid, '"/>\n');
    
    

    disp('-> Normals');
    fprintf(fid, ' 				<Normal vector="');
       
    for count = 1:m
        fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,4), surface_normals(count,5), surface_normals(count,6));
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,1), surface_normals(count,2), surface_normals(count,3));
        fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,1), surface_normals(count,2), surface_normals(count,3));
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', -surface_normals(count,4), -surface_normals(count,5), -surface_normals(count,6));        
    end 
    
    
    fprintf(fid, '"/>\n');    
    
    
    
    disp('-> Texture Coordinates');
    fprintf(fid, ' 				<TextureCoordinate point="');
       
    texture_scale = 0.5;
    
    for count = 1:m
        fprintf(fid, '0.4 %2.3f ', (count-1)*texture_scale);
        
        fprintf(fid, '0.0 %2.3f ', (count-1)*texture_scale);
        fprintf(fid, '1.0 %2.3f ', (count-1)*texture_scale);
        
        fprintf(fid, '0.6 %2.3f ', (count-1)*texture_scale);        
    end 
   
    fprintf(fid, '"/>\n');       
    
    
    
    fprintf(fid, '			</IndexedFaceSet>\n');
    fprintf(fid, '			<Appearance>\n');
    fprintf(fid, '				<Material diffuseColor="1.0 1.0 1.0"/>\n');
    fprintf(fid, '			</Appearance>\n');
    fprintf(fid, '		</Shape>\n');
    fprintf(fid, '		<Viewpoint description="View Object 0" orientation="1 0 0 0" position="0.000000 0.000000 2.500000"/>\n');
    fprintf(fid, '		<NavigationInfo type="EXAMINE"/>\n');
    fprintf(fid, '	</Scene>\n');
    fprintf(fid, '</X3D>\n');
catch
    fclose(fid);
    
    disp('Error caught.  Closed file');
end


fclose(fid);

disp('File written.');


%%
function generate_x3d_fence(fence, output_file)

disp('Generating Fence X3D File...');
disp('Generating Indices...');
m = size(fence,2);
indexed_face_set = zeros((m-1)*4*2*4, 1);
index_count = 1;

fence_height = 2;


for x = 0:(m-2)
    for y = 0:3
        indexed_face_set(index_count + 0) = x*8 + y*2 + 0;
        indexed_face_set(index_count + 1) = x*8 + y*2 + 1;
        indexed_face_set(index_count + 2) = x*8 + y*2 + 8;
        indexed_face_set(index_count + 3) = -1;

        indexed_face_set(index_count + 4) = x*8 + y*2 + 8;
        indexed_face_set(index_count + 5) = x*8 + y*2 + 1;
        indexed_face_set(index_count + 6) = x*8 + y*2 + 9;
        indexed_face_set(index_count + 7) = -1;     

        index_count = index_count + 8;
    end
end 



% disp('Generating Normals...');
% surface_normals = ones(m, 6);
% 
% for count = 1:(m-1)
% 
%     angle = atan2(fence(2,count+1) - fence(2,count), fence(1,count+1) - fence(1,count) );
%     
%     surface_normals(count, 1) = sin(angle-pi/2);
%     surface_normals(count, 2) = cos(angle-pi/2);
%     surface_normals(count, 3) = 0;
%     
%     angle = atan2(fence(5,count+1) - fence(5,count), fence(4,count+1) - fence(4,count));
%         
%     surface_normals(count, 4) = sin(angle-pi/2);
%     surface_normals(count, 5) = cos(angle-pi/2);
%     surface_normals(count, 6) = 0;
%      
% end
% 
% surface_normals(m, 1) = surface_normals(1, 1);
% surface_normals(m, 2) = surface_normals(1, 2);
% surface_normals(m, 3) = surface_normals(1, 3);
% surface_normals(m, 4) = surface_normals(1, 4);
% surface_normals(m, 5) = surface_normals(1, 5);
% surface_normals(m, 6) = surface_normals(1, 6);



disp('Writing File...');

fid = fopen(output_file, 'w');

try
    fprintf(fid, '<?xml version="1.0" encoding="utf-8"?>\n');
    fprintf(fid, '<X3D profile="Immersive" version="3.0">\n');
    fprintf(fid, '	<Scene>\n');
    fprintf(fid, '		<Shape>\n');
    
    disp('-> Indices');
    fprintf(fid, '			<IndexedFaceSet coordIndex="');
    
    for count = 1:size(indexed_face_set, 1)
        fprintf(fid, '%d ', indexed_face_set(count,1));
    end
        
    fprintf(fid, '">\n');

    disp('-> Vertices');
    fprintf(fid, ' 				<Coordinate point="');
       
    for count = 1:m
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(1,count), fence(2,count), fence(3,count));
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(1,count), fence(2,count), fence(3,count) + fence_height);
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(1,count), fence(2,count), fence(3,count) + fence_height);
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(1,count), fence(2,count), fence(3,count));
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(4,count), fence(5,count), fence(6,count));
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(4,count), fence(5,count), fence(6,count) + fence_height);
        
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(4,count), fence(5,count), fence(6,count) + fence_height);
        fprintf(fid, '%2.3f %2.3f %2.3f ', fence(4,count), fence(5,count), fence(6,count));
        
    end  
    
    fprintf(fid, '"/>\n');
    
    

%     disp('-> Normals');
%     fprintf(fid, ' 				<Normal vector="');
%        
%     for count = 1:m
%         fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,1), surface_normals(count,2), surface_normals(count,3));
%         fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,1), surface_normals(count,2), surface_normals(count,3));
%         
%         fprintf(fid, '%2.3f %2.3f %2.3f ', -surface_normals(count,1), -surface_normals(count,2), surface_normals(count,3));
%         fprintf(fid, '%2.3f %2.3f %2.3f ', -surface_normals(count,1), -surface_normals(count,2), surface_normals(count,3));
%         
%         fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,4), surface_normals(count,5), surface_normals(count,6));
%         fprintf(fid, '%2.3f %2.3f %2.3f ', surface_normals(count,4), surface_normals(count,5), surface_normals(count,6));
%         
%         fprintf(fid, '%2.3f %2.3f %2.3f ', -surface_normals(count,4), -surface_normals(count,5), surface_normals(count,6));
%         fprintf(fid, '%2.3f %2.3f %2.3f ', -surface_normals(count,4), -surface_normals(count,5), surface_normals(count,6));
%     end 
%     
%     
%     fprintf(fid, '"/>\n');    
    
    
    
    disp('-> Texture Coordinates');
    fprintf(fid, ' 				<TextureCoordinate point="');
       
    texture_scale = 1;
    
    for count = 1:m
        fprintf(fid, '%2.3f 0.0 ', (count-1)*texture_scale);
        fprintf(fid, '%2.3f 1.0 ', (count-1)*texture_scale);
        
        fprintf(fid, '%2.3f 1.0 ', (count-1)*texture_scale);
        fprintf(fid, '%2.3f 0.0 ', (count-1)*texture_scale);
        
        fprintf(fid, '%2.3f 0.0 ', (count-1)*texture_scale);
        fprintf(fid, '%2.3f 1.0 ', (count-1)*texture_scale);
        
        fprintf(fid, '%2.3f 1.0 ', (count-1)*texture_scale);
        fprintf(fid, '%2.3f 0.0 ', (count-1)*texture_scale);
        
    end 
   
    fprintf(fid, '"/>\n');       
    
    
    
    fprintf(fid, '			</IndexedFaceSet>\n');
    fprintf(fid, '			<Appearance>\n');
    fprintf(fid, '				<Material diffuseColor="1.0 1.0 1.0"/>\n');
    fprintf(fid, '			</Appearance>\n');
    fprintf(fid, '		</Shape>\n');
    fprintf(fid, '		<Viewpoint description="View Object 0" orientation="1 0 0 0" position="0.000000 0.000000 2.500000"/>\n');
    fprintf(fid, '		<NavigationInfo type="EXAMINE"/>\n');
    fprintf(fid, '	</Scene>\n');
    fprintf(fid, '</X3D>\n');
catch
    fclose(fid);
    
    disp('Error caught.  Closed file');
end


fclose(fid);

disp('File written.');

%%

function add_trees_to_scene(trees, path_to_scene_file)

% <Actor Color="1 1 1 1" Emissive="0" ID="actor.Tree_Trunk" Object="object.Tree_Trunk" Orientation="0 0 0" Position="1100 950 70" Scale="1 1 1" Specular="0 0 0 0">
%  <Actor Color="1 1 1 1" Emissive="0" Fog="true" ID="actor.Tree_Branches" Object="object.Tree_Branches" Orientation="0 0 0" Position="0 0 3.48" Priority="-3" Scale="1 1 1" Specular="0 0 0 0.1"/>
% </Actor>
%<!-- Start Trees -->
%<!-- End Trees -->

fid = fopen(path_to_scene_file, 'r');

scene_file = textscan(fid, '%s', 'Delimiter', '\n');

fclose(fid);

index_start = find(strcmp(scene_file{1}, '<!-- Start Trees -->'));
index_end = find(strcmp(scene_file{1}, '<!-- End Trees -->'));

new_file = cell(size(scene_file{1},1) - (index_end - index_start) + 1 + size(trees,2), 1);

new_file(1:index_start) = scene_file{1}(1:index_start);

for count = 1:size(trees,2)
    
    scale_size = rand()*2 + 3;
   
    new_file{index_start+1+(count-1) + 0} = sprintf('<Actor Color="1 1 1 1" Priority="-500" Emissive="0" ID="actor.Tree_Trunk_%d" Object="object.Tree_Trunk" Orientation="0 0 0" Position="%f %f %f" Scale="%f %f %f" Specular="0 0 0 0"/>', count, trees(1,count), trees(2,count), trees(3,count), scale_size, scale_size, scale_size);
    %new_file{index_start+1+(count-1)*3 + 1} = sprintf('<Actor Color="1 1 1 1" Emissive="0" Fog="true" ID="actor.Tree_Branches_%d" Object="object.Tree_Branches" Orientation="0 0 0" Position="0 0 3.48" Priority="-3" Scale="1 1 1" Specular="0 0 0 0.1"/>', count);
    %new_file{index_start+1+(count-1)*3 + 2} = sprintf('</Actor>'); 
    
end

new_file(index_start+1+(count):size(new_file,1)) = scene_file{1}(index_end:size(scene_file{1},1));

fid = fopen(path_to_scene_file, 'w');

for (count = 1:size(new_file,1))
    fwrite(fid, new_file{count});  
    fprintf(fid, '\n');
end


fclose(fid);






