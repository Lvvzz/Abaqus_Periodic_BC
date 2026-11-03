from abaqusGui import *
import copy
import json
import os
import tempfile
import time
import uuid
import sys
import traceback
import pbc_core as core


def ui_text(value):
    if sys.version_info[0]<3 and isinstance(value,unicode):
        return value.encode('mbcs','replace')
    return str(value)


class PeriodicBCDB(AFXDataDialog):
    ID_CHECK, ID_GENERATE, ID_REMOVE, ID_PRESET, ID_SELECT, ID_ADD, ID_SAVE, ID_DELETE, ID_UP, ID_DOWN = range(AFXDataDialog.ID_LAST+1, AFXDataDialog.ID_LAST+11)
    ID_PROGRESS = AFXDataDialog.ID_LAST+11

    def __init__(self, form):
        AFXDataDialog.__init__(self, form, 'PeriodicBC - 3D / Quasi-2D', self.CANCEL, DIALOG_ACTIONS_SEPARATOR, w=790, h=760)
        self.form = form
        self.cfg = copy.deepcopy(form.config)
        self.busy = False
        self.progress_timer = None
        self.progress_samples = 0
        self.editing_index = -1
        self.targets = {}
        self.fields = {}
        self.control_boxes = {}
        self.dialogs = []
        for message, handler in ((self.ID_CHECK,self.onCheck),(self.ID_GENERATE,self.onGenerate),
            (self.ID_REMOVE,self.onRemove),(self.ID_PRESET,self.onPreset),(self.ID_SELECT,self.onSelect),
            (self.ID_ADD,self.onAdd),(self.ID_SAVE,self.onSave),(self.ID_DELETE,self.onDelete),
            (self.ID_UP,self.onUp),(self.ID_DOWN,self.onDown)):
            function=getattr(handler,'im_func',None) or getattr(handler,'__func__',handler)
            FXMAPFUNC(self,SEL_COMMAND,message,function)
        FXMAPFUNC(self,SEL_UPDATE,self.ID_CHECK,PeriodicBCDB.onUpdateOptions)
        FXMAPFUNC(self,SEL_TIMEOUT,self.ID_PROGRESS,PeriodicBCDB.onProgressTimer)
        self.getActionButton(self.ID_CLICKED_CANCEL).setText('Close')
        scroll = FXScrollWindow(self,opts=LAYOUT_FILL_X|LAYOUT_FILL_Y)
        body = FXVerticalFrame(scroll,opts=LAYOUT_FILL_X,pl=8,pr=8,pt=8,pb=8)
        book = FXTabBook(body,opts=LAYOUT_FILL_X|LAYOUT_FILL_Y)
        FXTabItem(book,'Model / Loading')
        tab = FXVerticalFrame(book,opts=LAYOUT_FILL_X|LAYOUT_FILL_Y)
        group = FXGroupBox(tab,'Region (assembly coordinates; matching C3D8/C3D8R mesh)',FRAME_GROOVE|LAYOUT_FILL_X)
        self.text(group,'model','Model:',self.cfg['model'])
        self.text(group,'instances','Instances (comma-separated):',','.join(self.cfg['instances']))
        self.text(group,'tag','Configuration tag:',self.cfg['tag'])
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.combo(row,'mode','Mode:',('3D','QUASI2D'),self.cfg['mode'])
        self.combo(row,'thickness','Thickness:',('X','Y','Z'),self.cfg['thickness'])
        self.number(group,'tolerance','Tolerance (0 = automatic):',self.cfg['tolerance'])
        self.text(group,'report_directory','Report directory (blank = temp):',self.cfg.get('report_directory',''))
        group = FXGroupBox(tab,'Test and path',FRAME_GROOVE|LAYOUT_FILL_X)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.combo(row,'test','Test:',('TENSION','COMPRESSION','CREEP','SHEAR','CUSTOM'),self.cfg['test'])
        self.combo(row,'source','Material:',('UMAT','CREEP','ELASTIC'),self.cfg['source'])
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.combo(row,'organization','Path:',('SINGLE','MULTI'),self.cfg['organization'])
        self.combo(row,'step_source','Steps:',('CREATE','EXISTING'),self.cfg['step_source'])
        self.combo(group,'nlgeom','NLGEOM:',('ON','OFF'),'ON' if self.cfg['nlgeom'] else 'OFF')
        self.number(group,'cetol','Built-in creep CETOL:',self.cfg['cetol'])
        group = FXGroupBox(tab,'Preset - replaces the current path',FRAME_GROOVE|LAYOUT_FILL_X)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.combo(row,'axes','Axes:',('X','Y','Z','XY','XZ','YZ'),'X')
        self.combo(row,'preset_control','Control:',('H','P','MIXED'),'H')
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.number(row,'value1','Target A:',0.)
        self.number(row,'value2','Target B:',0.)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.number(row,'loading_time','Loading time:',1.)
        self.number(row,'hold_time','Hold time:',10000.)
        FXButton(group,'Apply preset',tgt=self,sel=self.ID_PRESET,opts=BUTTON_NORMAL)
        FXLabel(tab,'H = F-I (diagonal engineering extension); P = first Piola nominal stress.\nPositive = tension; negative = compression. Set negative targets for compression.\nUMAT time integration is user supplied. Built-in Creep uses Visco steps.',opts=JUSTIFY_LEFT|LAYOUT_FILL_X)

        FXTabItem(book,'Components / Stages')
        tab = FXVerticalFrame(book,opts=LAYOUT_FILL_X|LAYOUT_FILL_Y)
        group = FXGroupBox(tab,'Control types - fixed for the entire history',FRAME_GROOVE|LAYOUT_FILL_X)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        for c in core.COMPONENTS:
            self.combo(row,'control'+c,c,('H','P'),self.cfg['controls'][c])
        FXLabel(group,'Lower-triangular F is fixed to zero. Quasi-2D ignores thickness shear components.\nZero P = free in that macro component; zero H = locked macro deformation.',opts=JUSTIFY_LEFT|LAYOUT_FILL_X)
        group = FXGroupBox(tab,'Ordered stages (targets are TOTAL values, not increments)',FRAME_GROOVE|LAYOUT_FILL_X)
        self.stage_list = FXList(group,5,tgt=self,sel=self.ID_SELECT,opts=LIST_SINGLESELECT|LAYOUT_FILL_X|LAYOUT_FIX_HEIGHT,h=100)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        for title,message in (('Add',self.ID_ADD),('Save row',self.ID_SAVE),('Delete',self.ID_DELETE),('Up',self.ID_UP),('Down',self.ID_DOWN)):
            FXButton(row,title,tgt=self,sel=message,opts=BUTTON_NORMAL)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.text(row,'stage_name','Name:',self.cfg['stages'][0]['name'],18)
        self.combo(row,'kind','Kind:',('LOAD','HOLD','UNLOAD'),'LOAD')
        self.number(group,'duration','Duration:',1.)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        for c in core.COMPONENTS:
            self.number(row,'target'+c,c+':',0.,7)
        self.text(group,'amplitude','Existing amplitude (blank = generated linear path):','',32)
        row = FXHorizontalFrame(group,opts=LAYOUT_FILL_X)
        self.number(row,'initial','Initial:',.01,9)
        self.number(row,'minimum','Minimum:',1.e-5,9)
        self.number(row,'maximum','Maximum:',.02,9)
        self.integer(group,'increments','Maximum increments:',10000)
        FXLabel(tab,'HOLD inherits the preceding targets. UNLOAD uses the targets you enter.\nSINGLE combines rows into one step with generated amplitudes and a global maxInc.\nEXISTING: stage names/durations must match consecutive steps starting after Initial.\nExisting amplitude names are supported only for MULTI, zero-start transitions.',opts=JUSTIFY_LEFT|LAYOUT_FILL_X)

        FXTabItem(book,'Checks / Report')
        tab = FXVerticalFrame(book,opts=LAYOUT_FILL_X|LAYOUT_FILL_Y)
        self.report = FXText(tab,opts=TEXT_READONLY|LAYOUT_FILL_X|LAYOUT_FILL_Y|LAYOUT_FIX_HEIGHT,h=420)
        self.report.setText('Run Check before Generate.\nThe plugin will not edit materials or submit jobs.\nActive user BCs, loads, constraints and interactions are conservatively rejected.\nUse a clean model copy for the first application.\nQuasi-2D requires matching through-thickness columns and compatible microstructure.')
        actions = FXHorizontalFrame(self,opts=LAYOUT_FILL_X)
        self.buttons = []
        for label,message in (('Check / Preview',self.ID_CHECK),('Generate',self.ID_GENERATE),('Remove this configuration',self.ID_REMOVE)):
            self.buttons.append(FXButton(actions,label,tgt=self,sel=message,opts=BUTTON_NORMAL))
        group = FXGroupBox(self,'Processing progress',FRAME_GROOVE|LAYOUT_FILL_X)
        self.stage_label = FXLabel(group,'Ready',opts=JUSTIFY_LEFT|LAYOUT_FILL_X)
        self.count_label = FXLabel(group,'Items: 0/0 (0%)',opts=JUSTIFY_LEFT|LAYOUT_FILL_X)
        self.refresh_stages(0)

    def text(self,parent,key,label,value,columns=36):
        if sys.version_info[0]<3 and isinstance(value,unicode): value=value.encode('mbcs')
        target=AFXStringKeyword(None,key,True,str(value)); self.targets[key]=target
        self.fields[key]=AFXTextField(parent,columns,label,target,0,opts=AFXTEXTFIELD_STRING)

    def number(self,parent,key,label,value,columns=12):
        target=AFXFloatKeyword(None,key,True,float(value)); self.targets[key]=target
        self.fields[key]=AFXTextField(parent,columns,label,target,0)

    def integer(self,parent,key,label,value):
        target=AFXIntKeyword(None,key,True,int(value)); self.targets[key]=target
        AFXTextField(parent,12,label,target,0)

    def combo(self,parent,key,label,options,value):
        target=AFXStringKeyword(None,key,True,str(value)); self.targets[key]=target
        box=AFXComboBox(parent,14,len(options),label,target,0)
        for item in options: box.appendItem(item)
        self.control_boxes[key]=box

    def onUpdateOptions(self,sender,sel,ptr):
        if not self.busy:
            active=core.active_components(dict(mode=self.val('mode'),thickness=self.val('thickness')))
            for c in core.COMPONENTS:
                enabled=c in active
                self.control_boxes['control'+c].enable() if enabled else self.control_boxes['control'+c].disable()
                field=self.fields['target'+c]
                field.enable() if enabled and self.val('kind')!='HOLD' else field.disable()
            self.fields['cetol'].enable() if self.val('source')=='CREEP' else self.fields['cetol'].disable()
        return 1

    def val(self,key):
        return self.targets[key].getValue()

    def basic(self):
        for key in ('model','tag','mode','thickness','tolerance','test','source','organization','step_source','cetol','report_directory'):
            self.cfg[key]=self.val(key)
        self.cfg['model']=self.cfg['model'].strip()
        self.cfg['instances']=[n.strip() for n in self.val('instances').split(',') if n.strip()]
        self.cfg['nlgeom']=self.val('nlgeom')=='ON'
        for c in core.COMPONENTS: self.cfg['controls'][c]=self.val('control'+c)

    def save_row(self):
        current=self.editing_index
        if current < 0: return
        self.cfg['stages'][current]=dict(
            name=self.val('stage_name').strip(),kind=self.val('kind'),duration=self.val('duration'),
            targets=dict((c,self.val('target'+c)) for c in core.COMPONENTS),
            initial=self.val('initial'),minimum=self.val('minimum'),maximum=self.val('maximum'),
            increments=self.val('increments'),amplitude=self.val('amplitude').strip())

    def refresh_stages(self,current):
        self.stage_list.clearItems()
        for index,row in enumerate(self.cfg['stages']):
            self.stage_list.appendItem('%d. %s | %s | duration=%g' % (index+1,row['name'],row['kind'],row['duration']))
        if self.cfg['stages']:
            current=max(0,min(current,len(self.cfg['stages'])-1))
            self.stage_list.setCurrentItem(current)
            self.load_row(current)

    def load_row(self,current):
        if current<0: return
        self.editing_index=current
        row=self.cfg['stages'][current]
        for key in ('kind','duration','initial','minimum','maximum','increments','amplitude'):
            self.targets[key].setValue(row.get(key,''))
        self.targets['stage_name'].setValue(row['name'])
        for c in core.COMPONENTS: self.targets['target'+c].setValue(row['targets'][c])

    def onSelect(self,sender,sel,ptr):
        current=self.stage_list.getCurrentItem()
        self.save_row(); self.load_row(current); return 1

    def onSave(self,sender,sel,ptr):
        index=self.stage_list.getCurrentItem(); self.save_row(); self.refresh_stages(index); return 1

    def onAdd(self,sender,sel,ptr):
        self.save_row()
        row=core.stage('Stage%d'%(len(self.cfg['stages'])+1),1.)
        row['targets']=copy.deepcopy(self.cfg['stages'][-1]['targets'])
        self.cfg['stages'].append(row); self.refresh_stages(len(self.cfg['stages'])-1); return 1

    def onDelete(self,sender,sel,ptr):
        if len(self.cfg['stages'])>1:
            index=self.stage_list.getCurrentItem(); del self.cfg['stages'][index]; self.refresh_stages(index)
        return 1

    def move(self,offset):
        self.save_row(); i=self.stage_list.getCurrentItem(); j=i+offset
        if 0<=j<len(self.cfg['stages']):
            self.cfg['stages'][i],self.cfg['stages'][j]=self.cfg['stages'][j],self.cfg['stages'][i]
            self.refresh_stages(j)
        return 1

    def onUp(self,sender,sel,ptr): return self.move(-1)
    def onDown(self,sender,sel,ptr): return self.move(1)

    def onPreset(self,sender,sel,ptr):
        try:
            self.basic()
            self.cfg=core.preset(self.cfg,self.val('test'),self.val('axes'),self.val('value1'),self.val('value2'),self.val('preset_control'),self.val('loading_time'),self.val('hold_time'))
            for c in core.COMPONENTS: self.targets['control'+c].setValue(self.cfg['controls'][c])
            self.refresh_stages(0)
        except Exception as error: showAFXErrorDialog(self,str(error))
        return 1

    def onCheck(self,sender,sel,ptr): return self.execute('CHECK')
    def onGenerate(self,sender,sel,ptr): return self.execute('GENERATE')
    def onRemove(self,sender,sel,ptr): return self.execute('REMOVE')

    def onProgressTimer(self,sender,sel,ptr):
        self.progress_timer=None
        if not self.busy: return 1
        try:
            with open(self.progress_path,'rb') as stream: status=json.loads(stream.read())
            if status.get('state')=='PROGRESS':
                self.stage_label.setText(ui_text(status['stage']))
                total=status['total']; current=status['current']
                self.count_label.setText('Items: %d/%d (%d%%)'%(current,total,int(100.*current/total) if total else 0))
                self.progress_samples+=1
                getAFXApp().repaint()
        except Exception:
            pass
        self.progress_timer=getAFXApp().addTimeout(100,self,self.ID_PROGRESS)
        return 1

    def execute(self,action):
        if self.busy: return 1
        try:
            self.basic(); self.save_row()
            config=core.validate_config(self.cfg) if action!='REMOVE' else copy.deepcopy(self.cfg)
            self.form.config=copy.deepcopy(config)
            directory=config.get('report_directory','').strip() or tempfile.gettempdir()
            if not os.path.isdir(directory): raise ValueError('Report directory does not exist.')
            report_path=os.path.join(directory,'PeriodicBC_%s_%s.json'%(config['tag'],uuid.uuid4().hex[:10]))
            self.busy=True
            self.progress_path=report_path+'.status'
            self.progress_timer=getAFXApp().addTimeout(100,self,self.ID_PROGRESS)
            for button in self.buttons: button.disable()
            self.stage_label.setText(action+' - waiting for Abaqus kernel')
            self.count_label.setText('Items: 0/0 (0%)')
            self.repaint()
            json_kwargs={'encoding':'mbcs'} if sys.version_info[0]<3 else {}
            command='import PeriodicBC_kernel\nPeriodicBC_kernel.dispatch(%r, %r, %r)'%(action,json.dumps(config,ensure_ascii=True,**json_kwargs),report_path)
            sendCommand(ui_text(command))
            started=time.time()
            while time.time()-started<3600:
                try:
                    with open(report_path+'.status','rb') as stream: status=json.loads(stream.read())
                except Exception: status={}
                self.stage_label.setText(ui_text(status.get('stage','Waiting for kernel...')))
                total=status.get('total',0); current=status.get('current',0)
                self.count_label.setText('Items: %d/%d (%d%%)'%(current,total,int(100.*current/total) if total else 0))
                if status.get('state')=='ERROR':
                    raise RuntimeError(status.get('message','')+'\n'+status.get('traceback',''))
                if status.get('state')=='DONE':
                    with open(report_path,'rb') as stream: report=json.loads(stream.read())
                    self.report.setText(ui_text(json.dumps(report,indent=2,ensure_ascii=True)+'\nReport: '+report_path))
                    self.stage_label.setText(action+' completed - see Checks / Report')
                    break
                getAFXApp().runWhileEvents()
                time.sleep(.1)
            else: raise RuntimeError('Kernel did not finish within 60 minutes. Check kernel messages before retrying.')
        except Exception as error:
            self.stage_label.setText('Failed - see Checks / Report')
            message=ui_text(error)+'\n'+traceback.format_exc()
            self.report.setText(message); showAFXErrorDialog(self,message)
        finally:
            self.busy=False
            if self.progress_timer is not None:
                getAFXApp().removeTimeout(self.progress_timer)
                self.progress_timer=None
            for button in self.buttons: button.enable()
        return 1
