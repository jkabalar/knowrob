/*
  Copyright (C) 2020 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(mongo_tf,
    [
      mng_tf_current/4,
      mng_tf_latest_before/5,
      mng_tf_earliest_after/5,
      mng_tf_trajectory/5,
      mng_tf_interpolate/4
    ]).
/** <module> Looking up tf transforms in a mongo DB

@author Daniel Beßler
@license BSD
*/

:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/mongo')).

%% mng_tf_current(+DB, +ObjFrame, ?Pose, ?PoseTime) is nondet.
%
% Same as mng_tf_latest_before/5 but requests the latest
% record before the current time.
%
% @param DB Name of the database.
% @param ObjFrame The frame for which to look up the transform.
% @param Pose A list [RefFrame,ObjFrame,[X,Y,Z],[QX,QY,QZ,QW]].
% @param PoseTime The time for which the transform was recorded.
%
mng_tf_current(DB, ObjFrame, Pose, PoseTime) :-
  current_time(Now),
  mng_tf_latest_before(DB, ObjFrame, Now, Pose, PoseTime).

%% mng_tf_latest_before(+DB, +ObjFrame, +RequestedTime, ?Pose, ?PoseTime) is nondet.
%
% Read the latest record of some frame(s) in mongo DB
% from the collection named *tf*, and
% that were recorded before the given time.
% *ObjFrame* may either be one frame name atom, or
% a list of such.
% In case *ObjFrame* is a list, a choicepoint will be
% generated for each transform.
% Otherwise the predicate is semidet yielding exatly one
% pose or none in case the frame is not defined.
%
% Note: It is not possible to request the transform
% being expressed relative to a specific parent frame,
% transforms are yielded as stored in the DB.
%
% @param DB Name of the database.
% @param ObjFrame The frame for which to look up the transform.
% @param RequestedTime The time for which the transform is requested.
% @param Pose A list [RefFrame,ObjFrame,[X,Y,Z],[QX,QY,QZ,QW]].
% @param PoseTime The time for which the transform was recorded.
%
mng_tf_latest_before(DB, ObjFrame, RequestedTime, Pose, ActualTime) :-
  mng_tf_query_(DB, ObjFrame, RequestedTime, Pose, ActualTime, mng_tf_cursor_latest_).

%% mng_tf_earliest_after(+DB, +ObjFrame, +RequestedTime, ?Pose, ?ActualTime) is nondet.
%
% Same as mng_tf_latest_before/5 but requests the earliest
% record after the given time instead.
%
% @param DB Name of the database.
% @param ObjFrame The frame for which to look up the transform.
% @param RequestedTime The time for which the transform is requested.
% @param Pose A list [RefFrame,ObjFrame,[X,Y,Z],[QX,QY,QZ,QW]].
% @param ActualTime The time for which the transform was recorded.
%
mng_tf_earliest_after(DB, ObjFrame, RequestedTime, Pose, ActualTime) :-
  mng_tf_query_(DB, ObjFrame, RequestedTime, Pose, ActualTime, mng_tf_cursor_earliest_).

%%
mng_tf_query_(DB, ObjFrame, RequestedTime, Pose, ActualTime, CursorFactory) :-
  atom(ObjFrame),!,
  setup_call_cleanup(
    call(CursorFactory, DB, RequestedTime, Cursor),
    (
      mng_cursor_filter(Cursor,
       ['transforms', ['$elemMatch', ['child_frame_id', ['$eq', string(ObjFrame)]]]]),
      mng_cursor_limit(Cursor,1),
      mng_tf_cursor_read_(Cursor,ActualTime,Pose,[ObjFrame])
    ),
    mng_cursor_destroy(Cursor)
  ).

mng_tf_query_(DB, RequestedFrames, RequestedTime, Pose, ActualTime, CursorFactory) :-
  is_list(RequestedFrames),!,
  mng_tf_query_list_(DB, RequestedFrames, RequestedTime, Pose, ActualTime, CursorFactory).

%%
mng_tf_query_list_(_, [], _, _, _, _) :- !, fail.
mng_tf_query_list_(DB, RequestedFrames, RequestedTime, Pose, ActualTime, CursorFactory) :-
  %% read cursor poses
  RequestedFrames=[NextFrame|_],
  setup_call_cleanup(
    call(CursorFactory, DB, RequestedTime, Cursor),
    ( mng_cursor_filter(Cursor,
        ['transforms.child_frame_id', ['$eq', string(NextFrame)]]),
      % limit to one document, but the document has an array of transforms
      % that at least contains a transform for *NextFrame*
      % FIXME: this seems dangerous, the mongo tf logger does not ensure
      %         that the same groups are published together all the time.
      %         does not seem like getting around doing n queries for n
      %         frames will be as simple as this....
      mng_cursor_limit(Cursor,1),
      findall([T,P], mng_tf_cursor_read_(Cursor,T,P,RequestedFrames), CursorPoses)
    ),
    mng_cursor_destroy(Cursor)
  ),
  %%
  length(CursorPoses,NumPoses),
  ( NumPoses>0 -> true ; (
    print_message(warning, mng(tf(unknown_frame(NextFrame,RequestedTime)))),
    fail
  )),
  %%
  ( member([ActualTime,Pose],CursorPoses) ; (
    findall(F, (
      member(F,RequestedFrames),
      \+ member([_,[_,F,_,_]],CursorPoses)
    ), RemainingFrames),
    mng_tf_query_list_(DB, RemainingFrames, RequestedTime, Pose, ActualTime, CursorFactory)
  )).

%% mng_tf_trajectory(+DB, +ObjFrame, +Begin, +End, -StampedPoses) is semidet.
%
%
mng_tf_trajectory(DB, ObjFrame, Begin, End, StampedPoses) :-
  setup_call_cleanup(
    mng_tf_cursor_earliest_(DB, Begin, Cursor),
    ( mng_cursor_filter(Cursor,
        ['transforms.header.stamp', ['$lt', time(End)]]),
      mng_cursor_filter(Cursor,
        ['transforms', ['$elemMatch', ['child_frame_id', ['$eq', string(ObjFrame)]]]]),
      findall([T,P], (
        (T=Begin,mng_tf_interpolate(DB,ObjFrame,T,P));
        mng_tf_cursor_read_(Cursor,T,P);
        (T=End,  mng_tf_interpolate(DB,ObjFrame,T,P))
      ), StampedPoses)
    ),
    mng_cursor_destroy(Cursor)
  ).

%% mng_tf_interpolate(+DB, +ObjFrame, +RequestedTime, -Pose) is semidet.
%
%
% TODO: mongo 3.2 can do it in a single query:
%   https://stackoverflow.com/questions/13275491/mongodb-find-document-with-closest-integer-value
% FIXME: probably best to avoid interpolating in huge time frames.
%         could be an object was not tracked, and re-perceived
%         at some other location.
mng_tf_interpolate(DB, ObjFrame, RequestedTime, Pose) :-
  atom(ObjFrame),!,
  mng_tf_latest_before(DB, ObjFrame, RequestedTime, Pose0, Time0),
  (  mng_tf_earliest_after(DB, ObjFrame, RequestedTime, Pose1, Time1)
  -> mng_tf_interpolate_([Time0,Pose0],[Time1,Pose1],RequestedTime,Pose);
  (  Pose=Pose0 )).

mng_tf_interpolate(DB, ObjFrames, RequestedTime, Pose) :-
  is_list(ObjFrames),!,
  findall([LateTime,Late],
    mng_tf_latest_before(DB, ObjFrames, RequestedTime, Late, LateTime),
    LatePoses),
  mng_tf_earliest_after(DB, ObjFrames, RequestedTime, Early, Time1),
  %% find latest with same frame
  Early=[_,Frame,_,_],
  Late=[_,Frame,_,_],
  member([Time0,Late],LatePoses),
  %%
  mng_tf_interpolate_([Time0,Late],[Time1,Early],RequestedTime,Pose).

%%
mng_tf_interpolate_([T0,P0], [T1,P1], RequestedTime, Pose) :-
  Factor is 1.0 - (RequestedTime - T0)/(T1 - T0),
  transform_interpolate(P0,P1,Factor,Pose).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Helper

%%
mng_tf_cursor_latest_(DB,Time,Cursor) :-
  mng_cursor_create(DB, tf, Cursor),
  mng_cursor_filter(Cursor,
    ['transforms.header.stamp', ['$lt', time(Time)]]),
  mng_cursor_descending(Cursor,
    'transforms.header.stamp').
%%
mng_tf_cursor_earliest_(DB,Time,Cursor) :-
  mng_cursor_create(DB, tf, Cursor),
  mng_cursor_filter(Cursor,
    ['transforms.header.stamp', ['$gt', time(Time)]]),
  mng_cursor_ascending(Cursor,
    'transforms.header.stamp').

%%
mng_doc_pose_(Doc,Time,Pose) :-
  get_dict(transforms, Doc, List),
  member(SubDoc,List),
  mng_tf_pose_(SubDoc,Time,Pose).

%%
mng_tf_pose_(Doc,Time,[ParentFrame,ObjFrame,[TX,TY,TZ],[QX,QY,QZ,QW]]) :-
  get_dict(header,Doc,Header),
  get_dict(transform,Doc,PoseDoc),
  get_dict(translation,PoseDoc,TDoc),
  get_dict(rotation,PoseDoc,QDoc),
  %%
  mng_get_dict(child_frame_id,Doc,ObjFrame),
  mng_get_dict(frame_id,Header,ParentFrame),
  mng_get_dict(stamp,Header,Time),
  %%
  mng_get_dict(x,TDoc,TX),
  mng_get_dict(y,TDoc,TY),
  mng_get_dict(z,TDoc,TZ),
  %%
  mng_get_dict(x,QDoc,QX),
  mng_get_dict(y,QDoc,QY),
  mng_get_dict(z,QDoc,QZ),
  mng_get_dict(w,QDoc,QW).

%%
mng_tf_cursor_read_(Cursor, PoseTime, Pose) :-
  mng_cursor_next(Cursor,Doc),
  ( mng_doc_pose_(Doc, PoseTime, Pose);
    mng_tf_cursor_read_(Cursor, PoseTime, Pose)
  ).

%%
mng_tf_cursor_read_(_, _, _, []) :- !, fail.
mng_tf_cursor_read_(Cursor, PoseTime, Pose, RequestedFrames) :-
  mng_cursor_next(Cursor,Doc),
  % read poses from next cursor document
  findall([T,P], (
    mng_doc_pose_(Doc,T,P),
    P=[_,Frame,_,_],
    once(member(Frame,RequestedFrames))
  ), Poses0),
  Poses0 \= [],
  % yield each pose
  ( member([PoseTime,Pose],Poses0) ;
  % and move on to next cursor position (if any)
  ( findall(F, (
      member(F,RequestedFrames),
      \+ member([_,[_,F,_,_]],Poses0)
    ), RemainingFrames),
    mng_tf_cursor_read_(Cursor, PoseTime, Pose, RemainingFrames)
  )).
